package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ser.std.EnumSetSerializer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.jfr.Enabled;
import org.littletonrobotics.junction.AutoLog;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SafeAKitLogger;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;
import static org.tahomarobotics.robot.shooter.Shooter.ShootMode.*;


class ShooterIO {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ShooterIO.class);

    private final TalonFX topShooterMotor;
    private final TalonFX bottomShooterMotor;
    private final TalonFX pivotMotor;

    private final StatusSignal<Double> topShooterVelocity;
    private final StatusSignal<Double> bottomShooterVelocity;
    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotVelocity;
    private final StatusSignal<Double> topShooterVoltage, bottomShooterVoltage;

    private final StatusSignal<Double> topShooterCurrent;
    private final StatusSignal<Double> bottomShooterCurrent;

    private final StatusSignal<Double> pivotCurrent;

    private final MotionMagicVoltage pivotPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage motorVelocity = new MotionMagicVelocityVoltage(SHOOTER_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage passVelocity = new MotionMagicVelocityVoltage(HIGH_PASS_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVelocity = new MotionMagicVelocityVoltage(IDLE_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage transferVelocity = new MotionMagicVelocityVoltage(TRANSFER_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage reverseIntakeVelocity = new MotionMagicVelocityVoltage(-REVERSE_INTAKE_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    protected double angle = 0.0;

    private Shooter.ShootMode shootingMode = SHOOTING;
    private boolean redundantShootingMode = false;
    private boolean idleMode = true;
    private boolean isZeroed = false;
    private boolean readyMode = false;

    private double targetShooterSpeed = 0.0;

    @AutoLog
    static class ShooterIOInputs {
        double angle = 0.0;
    }

    ShooterIO() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
        topShooterMotor = new TalonFX(RobotMap.LEFT_SHOOTER_MOTOR);
        bottomShooterMotor = new TalonFX(RobotMap.RIGHT_SHOOTER_MOTOR);

        configurator.configureTalonFX(pivotMotor, pivotMotorConfiguration, "pivot motor");
        configurator.configureTalonFX(topShooterMotor, shooterMotorConfiguration, bottomShooterMotor, false);

        topShooterVelocity = topShooterMotor.getVelocity();
        bottomShooterVelocity = bottomShooterMotor.getVelocity();
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();
        topShooterVoltage = topShooterMotor.getMotorVoltage();
        bottomShooterVoltage = bottomShooterMotor.getMotorVoltage();

        topShooterCurrent = topShooterMotor.getSupplyCurrent();
        bottomShooterCurrent = bottomShooterMotor.getSupplyCurrent();
        pivotCurrent = pivotMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                topShooterVelocity, bottomShooterVelocity, pivotVelocity, topShooterVoltage,
                topShooterCurrent, bottomShooterCurrent, pivotCurrent, bottomShooterVoltage
        );
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, bottomShooterMotor, topShooterMotor);
    }

    void periodic() {
        if (!redundantShootingMode) {
            if (readyMode) {
                switch (shootingMode) {
                    case SHOOTING -> enable();
                    case PASSING_LOW -> {
                        enable();
                        AmpArm.getInstance().setArmState(AmpArm.ArmState.PASSING);
                    }
                    case PASSING_HIGH -> enablePassHighSpeed();
                }
            } else {
                if (idleMode) idle();
                setShooterAngle(SHOOTER_COLLECT_PIVOT_ANGLE);
            }
        }

        SafeAKitLogger.recordOutput("Shooter/ShootMode", shootingMode);
        SafeAKitLogger.recordOutput("Shooter/ReadyMode", readyMode);
        SmartDashboard.putString("Shooter/ShootMode", shootingMode.name());
        SmartDashboard.putBoolean("Shooter/ReadyMode", readyMode);
    }

    // GETTERS

    double getTopShooterVelocity() {
        return topShooterVelocity.getValue();
    }

    double getBottomShooterVelocity() {
        return bottomShooterVelocity.getValue();
    }

    double getTopShooterCurrent() {
        return topShooterCurrent.getValueAsDouble();
    }

    double getBottomShooterCurrent() {
        return bottomShooterCurrent.getValueAsDouble();
    }

    double getTopShooterVoltage() {
        return topShooterVoltage.getValueAsDouble();
    }

    double getBottomShooterVoltage() {
        return bottomShooterVoltage.getValueAsDouble();
    }

    double getPivotPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(pivotPosition, pivotVelocity);
    }

    boolean isSpinningAtVelocity() {
        return Math.abs(targetShooterSpeed - getTopShooterVelocity()) < SHOOTER_SPEED_TOLERANCE
                && Math.abs(targetShooterSpeed - getBottomShooterVelocity()) < SHOOTER_SPEED_TOLERANCE;
    }

    boolean isAtAngle() {
        return Math.abs(angle - getPivotPosition()) < PIVOT_ANGLE_TOLERANCE;
    }

    boolean isReadyToShoot() {
        return isAtAngle() && isSpinningAtVelocity() && Chassis.getInstance().isReadyToShoot();
    }

    boolean inRedundantShootingMode(){return redundantShootingMode;}

    boolean inReadyMode() {
        return readyMode;
    }

    boolean inShootMode() {
        return shootingMode == SHOOTING;
    }

    boolean inPassingMode() {
        return shootingMode.equals(PASSING_HIGH) || shootingMode.equals(PASSING_LOW);
    }

    Shooter.ShootMode getShootMode() {
        return shootingMode;
    }

    double getPivotVelocity() {
        return pivotVelocity.getValue();
    }

    boolean isIdling() {
        return idleMode;
    }

    double getTopShooterMotorCurrent() {
        return topShooterCurrent.getValueAsDouble();
    }

    double getBottomShooterMotorCurrent() {
        return bottomShooterCurrent.getValueAsDouble();
    }

    double getPivotMotorCurrent() {
        return pivotCurrent.getValueAsDouble();
    }

    // SETTERS

    void setShooterAngle(double angle) {
        this.angle = angle;

        SafeAKitLogger.recordOutput("Shooter/Target Angle", angle);

        pivotMotor.setControl(pivotPositionControl.withPosition(angle));
    }

    void zero() {
        RobustConfigurator.retryConfigurator(() -> pivotMotor.setPosition(0.0),
                "Zeroed Pivot Motor",
                "FAILED TO SET PIVOT POSITION",
                "Retrying setting pivot position.");

        isZeroed = true;
    }

    public boolean isZeroed() {
        return isZeroed;
    }

    void transferToAmp() {
        targetShooterSpeed = transferVelocity.Velocity;

        topShooterMotor.setControl(transferVelocity);
    }

    void reverseIntake() {
        targetShooterSpeed = reverseIntakeVelocity.Velocity;

        topShooterMotor.setControl(reverseIntakeVelocity);
    }

    void setPivotVoltage(double voltage) {
        pivotMotor.setControl(new VoltageOut(voltage));
    }


    // STATES
    void enable() {
        targetShooterSpeed = motorVelocity.Velocity;

        topShooterMotor.setControl(motorVelocity);
    }

    void toggleReadyMode() {
        if (readyMode) {
            disableReadyMode();
        } else if (Indexer.getInstance().isCollected()) {
            enableReadyMode();
        }
    }

    void idle() {
        targetShooterSpeed = idleVelocity.Velocity;

        idleMode = true;
        topShooterMotor.setControl(idleVelocity);
    }

    void stop() {
        targetShooterSpeed = 0.0;

        idleMode = false;
        topShooterMotor.stopMotor();
    }

    void enableReadyMode() {
        readyMode = true;
    }
    void disableReadyMode() {
        readyMode = false;
        AmpArm.getInstance().setArmState(AmpArm.ArmState.STOW);
    }

    void toggleRedundantShootMode(){
        if (redundantShootingMode){
            disableRedundantShootMode();
            idle();
        } else if (Indexer.getInstance().isCollected()) {
            enableRedundantShootMode();
            enable();
        }
    }

    public void toggleIdle() {
        if (idleMode) {
            stop();
        } else {
            idle();
        }
    }

    void enableShootMode() {
        shootingMode = SHOOTING;
    }
    void enablePassHigh(){
        shootingMode = PASSING_HIGH;
    }

    void enablePassHighSpeed() {
        targetShooterSpeed = passVelocity.Velocity;
        topShooterMotor.setControl(passVelocity);
    }

    void enablePassLow() {
        shootingMode = PASSING_LOW;
    }

    void togglePassHigh() {
        if (getShootMode().equals(PASSING_HIGH)) {
            enableShootMode();
        } else {
            enablePassHigh();
        }
    }

    void togglePassLow() {
        if (getShootMode().equals(PASSING_LOW)) {
            enableShootMode();
        } else {
            enablePassLow();
        }
    }

    void enableRedundantShootMode(){
        redundantShootingMode = true;
    }

    void disableRedundantShootMode(){
        redundantShootingMode = false;
    }

    public void configureShooterForTeleop() {
        topShooterMotor.getConfigurator().apply(shooterMotorConfiguration.CurrentLimits
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_TELEOP)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_TELEOP));
        bottomShooterMotor.getConfigurator().apply(shooterMotorConfiguration.CurrentLimits
                .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_TELEOP)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_TELEOP));
    }

    // INPUTS

    void processInputs(ShooterIOInputs inputs) {
        setShooterAngle(inputs.angle);
    }

    void refreshSignals() {
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, topShooterVelocity, bottomShooterVelocity,
                topShooterCurrent, bottomShooterCurrent, pivotCurrent, topShooterVoltage, bottomShooterVoltage);
    }

    public double getTotalCurrent() {
        return Math.abs(topShooterCurrent.getValue()) + Math.abs(bottomShooterCurrent.getValue()) + Math.abs(pivotCurrent.getValue());
    }
}
