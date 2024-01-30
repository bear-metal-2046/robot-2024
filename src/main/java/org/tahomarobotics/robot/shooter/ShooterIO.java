package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;


class ShooterIO {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ShooterIO.class);

    private final TalonFX shooterMotor;
    private final TalonFX shooterMotorFollower;
    private final TalonFX pivotMotor;

    private final StatusSignal<Double> shooterVelocity;
    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotVelocity;

    private final MotionMagicVoltage pivotPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage motorVelocity = new MotionMagicVelocityVoltage(SHOOTER_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage transferVelocity = new MotionMagicVelocityVoltage(TRANSFER_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    protected double angle = 0.0;

    private boolean shootingMode = false;

    @AutoLog
    static class ShooterIOInputs {
        double angle = 0.0;
    }

    ShooterIO() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
        shooterMotor = new TalonFX(RobotMap.TOP_SHOOTER_MOTOR);
        shooterMotorFollower = new TalonFX(RobotMap.BOTTOM_SHOOTER_MOTOR);

        configurator.configureTalonFX(pivotMotor, pivotMotorConfiguration);
        configurator.configureTalonFX(shooterMotor, shooterMotorConfiguration);
        configurator.configureTalonFX(shooterMotorFollower, shooterMotorConfiguration);

        shooterVelocity = shooterMotor.getVelocity();
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                shooterVelocity, pivotPosition, pivotVelocity
        );
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, shooterMotorFollower, shooterMotor);
    }

    // GETTERS

    double getShooterVelocity() {
        return shooterVelocity.refresh().getValue();
    }

    double getPivotPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(pivotPosition.refresh(), pivotVelocity.refresh());
    }

    boolean isSpinningAtVelocity() {
        return Math.abs(SHOOTER_SPEED - getShooterVelocity()) < SHOOTER_SPEED_TOLERANCE;
    }

    boolean isAtAngle() {
        return Math.abs(angle - getPivotPosition()) < PIVOT_ANGLE_TOLERANCE;
    }

    boolean isReadyToShoot() {
        return isAtAngle() && isSpinningAtVelocity();
    }

    boolean inShootingMode() {
        return shootingMode;
    }

    double rotToSpeaker() {
        return Chassis.getInstance().getPose().getTranslation()
                .minus(SPEAKER_TARGET_POSITION.get()).getAngle().getRadians();
    }

    double angleToSpeaker() {
        Translation2d target = SPEAKER_TARGET_POSITION.get();
        double distance = Chassis.getInstance().getPose().getTranslation().getDistance(target);

        return Math.atan2(SPEAKER_HEIGHT_DIFF, distance) / (2 * Math.PI);
    }

    // SETTERS

    void setShooterAngle(double angle) {
        this.angle = angle;

        Logger.recordOutput("Shooter/Target Angle", angle);

        pivotMotor.setControl(pivotPositionControl.withPosition(angle));
    }

    void zero() { pivotMotor.setPosition(0.0); }

    public void transferToAmp() {
        shooterMotor.setControl(transferVelocity);
    }

    // STATES

    void enable() {
        shootingMode = true;

        shooterMotor.setControl(motorVelocity);
        shooterMotorFollower.setControl(motorVelocity);
    }

    void disable() {
        shootingMode = false;

        shooterMotor.stopMotor();
        shooterMotorFollower.stopMotor();
    }

    void toggleShootMode() {
        if (shootingMode) {
            disable();
        } else if (Indexer.getInstance().hasCollected()){
            enable();
        }
    }

    // INPUTS

    void processInputs(ShooterIOInputs inputs) {
        setShooterAngle(inputs.angle);
    }
}
