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


class DiffyShooterIO {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(DiffyShooterIO.class);

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;
    private final TalonFX pivotMotor;

    private final StatusSignal<Double> shooterVelocity;
    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotVelocity;
    private final StatusSignal<Double> motorVoltage;

    private final StatusSignal<Double> rightShooterMotorCurrent;
    private final StatusSignal<Double> leftShooterMotorCurrent;

    private final StatusSignal<Double> pivotCurrent;

    private final MotionMagicVoltage pivotPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage rightMotorVelocity = new MotionMagicVelocityVoltage(SHOOTER_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage leftMotorVelocity = new MotionMagicVelocityVoltage(SHOOTER_SPEED+LEFT_SHOOTER_OFFSET).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVelocity = new MotionMagicVelocityVoltage(IDLE_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage transferVelocity = new MotionMagicVelocityVoltage(TRANSFER_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage reverseIntakeVelocity = new MotionMagicVelocityVoltage(-REVERSE_INTAKE_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    protected double angle = 0.0;

    private boolean shootingMode = false;
    private boolean idleMode = true;
    private boolean isZeroed = false;

    private double targetShooterSpeed = 0.0;

    @AutoLog
    static class DiffyShooterIOInputs {
        double angle = 0.0;

    }

    DiffyShooterIO() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
        rightShooterMotor = new TalonFX(RobotMap.RIGHT_SHOOTER_MOTOR);
        leftShooterMotor = new TalonFX(RobotMap.LEFT_SHOOTER_MOTOR);

        configurator.configureTalonFX(pivotMotor, pivotMotorConfiguration, "pivot motor");
        configurator.configureTalonFX(rightShooterMotor, rightShooterMotorConfiguration, "shooter motor");
        configurator.configureTalonFX(leftShooterMotor, shooterMotorConfiguration, "shooter motor follower");

        shooterVelocity = rightShooterMotor.getVelocity();
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();
        motorVoltage = rightShooterMotor.getMotorVoltage();

        rightShooterMotorCurrent = rightShooterMotor.getSupplyCurrent();
        leftShooterMotorCurrent = leftShooterMotor.getSupplyCurrent();
        pivotCurrent = pivotMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                shooterVelocity, pivotPosition, pivotVelocity, motorVoltage,
                rightShooterMotorCurrent, leftShooterMotorCurrent, pivotCurrent
        );
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, leftShooterMotor, rightShooterMotor);
    }

    // GETTERS

    double getShooterVelocity() {
        return shooterVelocity.getValue();
    }
    double getPivotPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(pivotPosition, pivotVelocity);
    }

    boolean isSpinningAtVelocity() {
        return Math.abs(targetShooterSpeed - getShooterVelocity()) < SHOOTER_SPEED_TOLERANCE;
    }

    boolean isAtAngle() {
        return Math.abs(angle - getPivotPosition()) < PIVOT_ANGLE_TOLERANCE;
    }

    boolean isReadyToShoot() {
        //return isAtAngle() && isSpinningAtVelocity() && Chassis.getInstance().isReadyToShoot();
        System.out.println("target: " + targetShooterSpeed);
        System.out.println("actual: " + getShooterVelocity());
        return isSpinningAtVelocity();
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
        targetShooterSpeed = transferVelocity.Velocity;

        rightShooterMotor.setControl(transferVelocity);
        leftShooterMotor.setControl(transferVelocity);
    }

    void reverseIntake() {
        targetShooterSpeed = reverseIntakeVelocity.Velocity;

        rightShooterMotor.setControl(reverseIntakeVelocity);
        leftShooterMotor.setControl(reverseIntakeVelocity);
    }

    void setPivotVoltage(double voltage) {
        pivotMotor.setControl(new VoltageOut(voltage));
    }


    // STATES
    void enable() {
        targetShooterSpeed = rightMotorVelocity.Velocity;

        rightShooterMotor.setControl(rightMotorVelocity);
        leftShooterMotor.setControl(leftMotorVelocity);
    }

    void idle() {
        targetShooterSpeed = idleVelocity.Velocity;

        idleMode = true;
        rightShooterMotor.setControl(idleVelocity);
        leftShooterMotor.setControl(idleVelocity);
    }

    void stop() {
        targetShooterSpeed = 0.0;

        idleMode = false;
        rightShooterMotor.stopMotor();
        leftShooterMotor.stopMotor();
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

    void processInputs(DiffyShooterIOInputs inputs) {
        setShooterAngle(inputs.angle);
    }

    void refreshSignals() {
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, shooterVelocity,
                rightShooterMotorCurrent, leftShooterMotorCurrent, pivotCurrent);
    }

    public double getTotalCurrent() {
        return Math.abs(rightShooterMotorCurrent.getValue()) + Math.abs(leftShooterMotorCurrent.getValue()) + Math.abs(pivotCurrent.getValue());
    }
}
