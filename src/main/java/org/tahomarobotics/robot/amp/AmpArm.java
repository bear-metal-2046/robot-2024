package org.tahomarobotics.robot.amp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;

public class AmpArm extends SubsystemIF {
    private static final AmpArm INSTANCE = new AmpArm();

    private final TalonFX armMotor;
    private final TalonFX wristMotor;
    private final TalonFX rollersMotor;

    private final StatusSignal<Double> armPosition;
    private final StatusSignal<Double> wristPosition;
    private final StatusSignal<Double> armVelocity;
    private final StatusSignal<Double> wristVelocity;
    private final StatusSignal<Double> rollersVelocity;

    private final MotionMagicVoltage armControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage wristControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage rollerVelocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private ArmState armState = ArmState.STOW;
    private RollerState rollerState = RollerState.DISABLED;

    private double targetArmPosition, targetWristPosition;

    private AmpArm() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        armMotor = new TalonFX(RobotMap.ARM_MOTOR);
        wristMotor = new TalonFX(RobotMap.WRIST_MOTOR);
        rollersMotor = new TalonFX(RobotMap.ROLLERS_MOTOR);

        configurator.configureTalonFX(armMotor, armMotorConfiguration);
        configurator.configureTalonFX(wristMotor, wristMotorConfiguration);
        configurator.configureTalonFX(rollersMotor, rollerMotorConfiguration);

        armPosition = armMotor.getPosition();
        wristPosition = wristMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        wristVelocity = wristMotor.getVelocity();
        rollersVelocity = rollersMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                armPosition, wristPosition, armVelocity, wristVelocity, rollersVelocity
        );

        ParentDevice.optimizeBusUtilizationForAll(armMotor, wristMotor, rollersMotor);
    }


    // GETTERS

    public static AmpArm getInstance() {
        return INSTANCE;
    }

    private double getArmPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(armPosition.refresh(), armVelocity.refresh());
    }

    private double getWristPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(wristPosition.refresh(), wristVelocity.refresh());
    }

    public double getArmVelocity() {
        return armVelocity.refresh().getValue();
    }

    public double getWristVelocity() {
        return wristVelocity.refresh().getValue();
    }

    public double getRollersVelocity() {
        return rollersVelocity.refresh().getValue();
    }


    // STATE CONTROLLERY

    public void setArmState(ArmState state) {
        armState = state;

        setWristPosition(WRIST_MOVING_POSE);

        switch (state) {
            case STOW -> setArmPosition(ARM_STOW_POSE);
            case AMP -> setArmPosition(ARM_AMP_POSE);
            case SOURCE -> setArmPosition(ARM_SOURCE_POSE);
            case TRAP -> setArmPosition(ARM_TRAP_POSE);
        }
    }

    public void setArmPosition(double position) {
        targetArmPosition = MathUtil.clamp(position, ARM_MIN_POSE, ARM_MAX_POSE);
        armMotor.setControl(armControl.withPosition(targetArmPosition));
    }

    public void setRollerState(RollerState state) {
        rollerState = state;

        switch (state) {
            case PASSING -> rollersMotor.setControl(rollerVelocityControl.withVelocity(ShooterConstants.TRANSFER_VELOCITY));
            case SCORE -> rollersMotor.setControl(rollerVelocityControl.withVelocity(-ShooterConstants.TRANSFER_VELOCITY * 4));
            default -> rollersMotor.stopMotor();
        }
    }

    public void setWristPosition(double position) {
        targetWristPosition = position;
        wristMotor.setControl(wristControl.withPosition(targetWristPosition));
    }

    // STATE CHECKERY

    public boolean isStowed() {
        return armState == ArmState.STOW;
    }

    public boolean isAmp() {
        return armState == ArmState.AMP;
    }

    public boolean isTrap() {
        return armState == ArmState.TRAP;
    }

    public boolean isSource() {
        return armState == ArmState.SOURCE;
    }

    public boolean isPassing() {
        return rollerState == RollerState.PASSING;
    }

    public boolean isCollected() {
        return rollerState == RollerState.COLLECTED;
    }

    public boolean isScoring() {
        return rollerState == RollerState.SCORE;
    }

    public boolean isArmAtPosition() {
        return Math.abs(getArmPosition() - targetArmPosition) < POSITION_TOLERANCE;
    }

    public boolean isWristAtPosition() {
        return Math.abs(getWristPosition() - targetWristPosition) < POSITION_TOLERANCE;
    }

    public boolean isRollersAtVelocity() {
        return Math.abs(Math.abs(getRollersVelocity()) - ShooterConstants.TRANSFER_VELOCITY) < VELOCITY_TOLERANCE;
    }

    // PERIODIC

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(armPosition, wristPosition, armVelocity, wristVelocity, rollersVelocity);

        Logger.recordOutput("Amp Arm/Roller State", rollerState);
        Logger.recordOutput("Amp Arm/Arm State", armState);

        Logger.recordOutput("Amp Arm/Arm Position", getArmPosition());
        Logger.recordOutput("Amp Arm/Wrist Position", getWristPosition());
        Logger.recordOutput("Amp Arm/Arm Velocity", getArmVelocity());
        Logger.recordOutput("Amp Arm/Wrist Velocity", getWristVelocity());
        Logger.recordOutput("Amp Arm/Rollers Velocity", getRollersVelocity());
    }

    @Override
    public SubsystemIF initialize() {
        Commands.waitUntil(RobotState::isEnabled)
                .andThen(Commands.runOnce(() -> {
                    armMotor.setPosition(0.0);
                    wristMotor.setPosition(0.0);
                }))
                .ignoringDisable(true).schedule();

        return this;
    }

    public enum ArmState {
        STOW,
        AMP,
        SOURCE,
        TRAP
    }

    // STATES

    public enum RollerState {
        DISABLED,
        PASSING,
        COLLECTED,
        SCORE
    }


}
