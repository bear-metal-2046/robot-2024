package org.tahomarobotics.robot.amp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;

public class AmpArm extends SubsystemIF {
    private static final AmpArm INSTANCE = new AmpArm();

    private final TalonFX armMotor;
    private final TalonFX wristMotor;
    private final TalonFX rollerMotor;

    private final StatusSignal<Double> armPosition, wristPosition, rollerPosition, armVelocity, wristVelocity, rollerVelocity;
    private final StatusSignal<Double> armCurrent, wristCurrent, rollerCurrent;

    private final MotionMagicVoltage positionalControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage rollerPositionalControl = new MotionMagicVoltage(0.0).withSlot(1)
            .withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private ArmState armState = ArmState.STOW;
    private RollerState rollerState = RollerState.DISABLED;

    private double targetArmPosition, targetWristPosition, targetRollerPosition;

    private double energyUsed = 0;

    private AmpArm() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        armMotor = new TalonFX(RobotMap.ARM_MOTOR);
        wristMotor = new TalonFX(RobotMap.WRIST_MOTOR);
        rollerMotor = new TalonFX(RobotMap.ROLLERS_MOTOR);

        configurator.configureTalonFX(armMotor, armMotorConfiguration, "arm motor");
        configurator.configureTalonFX(wristMotor, wristMotorConfiguration, "wrist motor");
        configurator.configureTalonFX(rollerMotor, rollerMotorConfiguration, "roller motor");

        armPosition = armMotor.getPosition();
        wristPosition = wristMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        wristVelocity = wristMotor.getVelocity();
        rollerVelocity = rollerMotor.getVelocity();
        rollerPosition = rollerMotor.getPosition();

        armCurrent = armMotor.getSupplyCurrent();
        wristCurrent = wristMotor.getSupplyCurrent();
        rollerCurrent = rollerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                armPosition, wristPosition, armVelocity, wristVelocity, rollerVelocity,
                armCurrent, wristCurrent, rollerCurrent, rollerPosition
        );

        ParentDevice.optimizeBusUtilizationForAll(armMotor, wristMotor, rollerMotor);
    }

    // GETTERS

    public static AmpArm getInstance() {
        return INSTANCE;
    }

    public double getArmPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(armPosition, armVelocity);
    }

    private double getWristPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(wristPosition, wristVelocity);
    }

    public double getRollerPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(rollerPosition, rollerVelocity);
    }

    public double getArmVelocity() {
        return armVelocity.getValue();
    }

    public double getWristVelocity() {
        return wristVelocity.getValue();
    }

    public double getRollerVelocity() {
        return rollerVelocity.refresh().getValue();
    }

    public double getArmCurrent() {
        return armCurrent.getValueAsDouble();
    }

    // STATE CONTROL

    public void setArmState(ArmState state) {
        armState = state;

        switch (state) {
            case STOW -> {
                setArmPosition(ARM_STOW_POSE);
                setWristPosition(WRIST_STOW_POSE);
            }
            case AMP -> {
                setArmPosition(ARM_AMP_POSE);
                setWristPosition(WRIST_AMP_POSE);
            }
            case SOURCE -> {
                setArmPosition(ARM_SOURCE_POSE);
                setWristPosition(WRIST_SOURCE_POSE);
            }
            case TRAP -> {
                setArmPosition(ARM_TRAP_POSE);
                setWristPosition(WRIST_TRAP_POSE);
            }
            case CLIMB -> {
                setArmPosition(ARM_CLIMB_POSE);
                setWristPosition(WRIST_MOVING_POSE);
            }
        }
    }

    public void setArmPosition(double position) {
        targetArmPosition = MathUtil.clamp(position, ARM_MIN_POSE, ARM_MAX_POSE);
        armMotor.setControl(positionalControl.withPosition(targetArmPosition));
    }

    public void setWristPosition(double position) {
        targetWristPosition = position;
        wristMotor.setControl(positionalControl.withPosition(targetWristPosition));
    }

    public void setRollerState(RollerState state) {
        rollerState = state;

        switch (state) {
            case PASSING -> rollerMotor.setControl(velocityControl.withVelocity(ShooterConstants.TRANSFER_VELOCITY));
            case SCORE -> rollerMotor.setControl(velocityControl.withVelocity(-ShooterConstants.TRANSFER_VELOCITY * 4));
            case TRAP -> rollerMotor.setControl(velocityControl.withVelocity(-ShooterConstants.TRAP_VELOCITY));
            default -> rollerMotor.stopMotor();
        }
    }

    public void setRollerPosition(double position) {
        targetRollerPosition = position;
        rollerMotor.setControl(rollerPositionalControl.withPosition(targetRollerPosition));
    }

    // STATE CHECKING

    public boolean isArmAtStow() {
        return armState == ArmState.STOW;
    }

    public boolean isArmAtAmp() {
        return armState == ArmState.AMP;
    }

    public boolean isArmAtTrap() {
        return armState == ArmState.TRAP;
    }

    public boolean isArmAtSource() {
        return armState == ArmState.SOURCE;
    }

    public boolean hasRollerCollected() {
        return rollerState == RollerState.COLLECTED;
    }

    // AT POSITION/VELOCITY

    public boolean isArmAtPosition() {
        return Math.abs(getArmPosition() - targetArmPosition) < POSITION_TOLERANCE;
    }

    public boolean isWristAtPosition() {
        return Math.abs(getWristPosition() - targetWristPosition) < POSITION_TOLERANCE;
    }

    public boolean isRollerAtPosition() {
        return Math.abs(getRollerPosition() - targetRollerPosition) < POSITION_TOLERANCE;
    }

    // PERIODIC

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(armPosition, wristPosition, armVelocity, wristVelocity, rollerVelocity,
                armCurrent, wristCurrent, rollerCurrent);

        double voltage = RobotController.getBatteryVoltage();
        double totalCurrent = armCurrent.getValue() + wristCurrent.getValue() + rollerCurrent.getValue();
        energyUsed += totalCurrent * voltage * Robot.defaultPeriodSecs;

        SafeAKitLogger.recordOutput("Amp Arm/Roller State", rollerState);
        SafeAKitLogger.recordOutput("Amp Arm/Arm State", armState);

        SafeAKitLogger.recordOutput("Amp Arm/Arm Position", getArmPosition());
        SafeAKitLogger.recordOutput("Amp Arm/Wrist Position", getWristPosition());
        SafeAKitLogger.recordOutput("Amp Arm/Roller Position", getRollerPosition());
        SafeAKitLogger.recordOutput("Amp Arm/Arm Velocity", getArmVelocity());
        SafeAKitLogger.recordOutput("Amp Arm/Wrist Velocity", getWristVelocity());
        SafeAKitLogger.recordOutput("Amp Arm/Roller Velocity", getRollerVelocity());

        SafeAKitLogger.recordOutput("Amp Arm/Total Current", totalCurrent);
        SafeAKitLogger.recordOutput("Amp Arm/Arm Current", getArmCurrent());
        SafeAKitLogger.recordOutput("Amp Arm/Wrist Current", wristCurrent.getValueAsDouble());
        SafeAKitLogger.recordOutput("Amp Arm/Roller Current", rollerCurrent.getValueAsDouble());
        SafeAKitLogger.recordOutput("Amp Arm/Energy Used", getEnergyUsed());
    }

    @Override
    public SubsystemIF initialize() {
        Commands.waitUntil(RobotState::isEnabled)
                .andThen(Commands.runOnce(() -> {
                    armMotor.setPosition(ARM_STOW_POSE);
                    wristMotor.setPosition(WRIST_STOW_POSE);
                    setArmPosition(ARM_STOW_POSE);
                    setWristPosition(WRIST_STOW_POSE);
                }))
                .ignoringDisable(true).schedule();

        return this;
    }

    //STATES

    public enum ArmState {
        TRAP,
        STOW,
        AMP,
        SOURCE,
        CLIMB
    }

    public enum RollerState {
        DISABLED,
        PASSING,
        COLLECTED,
        TRAP,
        SCORE
    }

    @Override
    public double getEnergyUsed() {
        return energyUsed / 1000d;
    }
}
