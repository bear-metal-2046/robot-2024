package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SafeAKitLogger;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;


class ShooterIO {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ShooterIO.class);

    private final TalonFX shooterMotor;
    private final TalonFX shooterMotorFollower;
    private final TalonFX pivotMotor;

    private final StatusSignal<Double> shooterVelocity;
    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotVelocity;
    private final StatusSignal<Double> motorVoltage;

    private final StatusSignal<Double> shooterMotorCurrent;
    private final StatusSignal<Double> shooterMotorFollowerCurrent;

    private final StatusSignal<Double> pivotCurrent;

    private final MotionMagicVoltage pivotPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage motorVelocity = new MotionMagicVelocityVoltage(SHOOTER_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVelocity = new MotionMagicVelocityVoltage(IDLE_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage transferVelocity = new MotionMagicVelocityVoltage(TRANSFER_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage reverseIntakeVelocity = new MotionMagicVelocityVoltage(-TRANSFER_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    protected double angle = 0.0;

    private boolean shootingMode = false;
    private boolean idleMode = true;
    private boolean isZeroed = false;

    @AutoLog
    static class ShooterIOInputs {

        double angle = 0.0;

    }
    ShooterIO() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
        shooterMotor = new TalonFX(RobotMap.TOP_SHOOTER_MOTOR);
        shooterMotorFollower = new TalonFX(RobotMap.BOTTOM_SHOOTER_MOTOR);

        configurator.configureTalonFX(pivotMotor, pivotMotorConfiguration, "pivot motor");
        configurator.configureTalonFX(shooterMotor, shooterMotorConfiguration, "shooter motor");
        configurator.configureTalonFX(shooterMotorFollower, shooterMotorConfiguration, "shooter motor follower");

        shooterVelocity = shooterMotor.getVelocity();
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();
        motorVoltage = shooterMotor.getMotorVoltage();

        shooterMotorCurrent = shooterMotor.getSupplyCurrent();
        shooterMotorFollowerCurrent = shooterMotorFollower.getSupplyCurrent();
        pivotCurrent = pivotMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                shooterVelocity, pivotPosition, pivotVelocity, motorVoltage,
                shooterMotorCurrent, shooterMotorFollowerCurrent, pivotCurrent
        );
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, shooterMotorFollower, shooterMotor);
    }

    // GETTERS

    double getShooterVelocity() {
        return shooterVelocity.getValue();
    }
    double getPivotPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(pivotPosition, pivotVelocity);
    }

    boolean isSpinningAtVelocity() {
        return Math.abs(SHOOTER_SPEED - getShooterVelocity()) < SHOOTER_SPEED_TOLERANCE;
    }

    boolean isAtAngle() {
        return Math.abs(angle - getPivotPosition()) < PIVOT_ANGLE_TOLERANCE;
    }

    boolean isReadyToShoot() {
        return isAtAngle() && isSpinningAtVelocity() && Chassis.getInstance().isReadyToShoot();
    }

    boolean inShootingMode() {
        return shootingMode;
    }

    double getPivotVelocity() {
        return pivotVelocity.getValue();
    }

    boolean isIdling() {
        return idleMode;
    }

    // SETTERS

    void setShooterAngle(double angle) {
        this.angle = angle;

        SafeAKitLogger.recordOutput("Shooter/Target Angle", angle);

        pivotMotor.setControl(pivotPositionControl.withPosition(angle));
    }

    void zero() {
        pivotMotor.setPosition(0.0);
        isZeroed = true;
    }

    public boolean isZeroed() {
        return isZeroed;
    }

    void transferToAmp() {
        shooterMotor.setControl(transferVelocity);
        shooterMotorFollower.setControl(transferVelocity);
    }

    void reverseIntake() {
        shooterMotor.setControl(reverseIntakeVelocity);
        shooterMotorFollower.setControl(reverseIntakeVelocity);
    }

    void setPivotVoltage(double voltage) {
        pivotMotor.setControl(new VoltageOut(voltage));
    }


    // STATES
    void enable() {
        shooterMotor.setControl(motorVelocity);
        shooterMotorFollower.setControl(motorVelocity);
    }

    void idle() {
        idleMode = true;
        shooterMotor.setControl(idleVelocity);
        shooterMotorFollower.setControl(idleVelocity);
    }

    void stop() {
        idleMode = false;
        shooterMotor.stopMotor();
        shooterMotorFollower.stopMotor();
    }

    void toggleShootMode() {
        if (shootingMode) {
            disableShootMode();
            idle();
        } else if (Indexer.getInstance().hasCollected()){
            enableShootMode();
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
        shootingMode = true;
    }

    void disableShootMode() {
        shootingMode = false;
    }

    // INPUTS

    void processInputs(ShooterIOInputs inputs) {
        setShooterAngle(inputs.angle);
    }

    void refreshSignals() {
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, shooterVelocity,
                shooterMotorCurrent, shooterMotorFollowerCurrent, pivotCurrent);
    }

    public double getTotalCurrent() {
        return Math.abs(shooterMotorCurrent.getValue()) + Math.abs(shooterMotorFollowerCurrent.getValue()) + Math.abs(pivotCurrent.getValue());
    }
}
