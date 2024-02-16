package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLog;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.ToggledOutputs;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;


class ShooterIO implements ToggledOutputs {
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
    private final MotionMagicVelocityVoltage reverseIntakeVelocity = new MotionMagicVelocityVoltage(-TRANSFER_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    protected double angle = 0.0;
    protected double distance = 0.0;
    protected double prevRadVelocity = 0;
    protected double prevTime = 0;

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
        return isAtAngle() && isSpinningAtVelocity();
    }

    boolean inShootingMode() {
        return shootingMode;
    }

    void angleToSpeaker(double radialComponent) {

        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - prevTime;

        double acc = (radialComponent - prevRadVelocity) / elapsedTime;

        Translation2d target = SPEAKER_TARGET_POSITION.get();
        distance = Chassis.getInstance().getPose().getTranslation().getDistance(target) + SHOOTER_PIVOT_OFFSET.getX();


        // This was for tying to compensate for the shooter being off when moving back and forth,
        // but it turns out that the shooter is lagging behind the commands angle, so no matter how
        // much time we try compensate, its always going to lag behind.
        distance = (0.5 * acc * TIME_SHOT_OFFSET * TIME_SHOT_OFFSET) + (radialComponent * TIME_SHOT_OFFSET) + distance;

        prevTime = currentTime;
        prevRadVelocity = radialComponent;

        setShooterAngle(0.04875446 + (0.201136 - 0.04875446)/(1 + Math.pow((distance/2.019404), 2.137465)));
    }

    // SETTERS

    void setShooterAngle(double angle) {
        this.angle = angle;

        recordOutput("Shooter/Target Angle", angle);

        pivotMotor.setControl(pivotPositionControl.withPosition(angle));
    }

    double getDistance() {
        return distance;
    }

    void zero() { pivotMotor.setPosition(0.0); }

    public void transferToAmp() {
        shooterMotor.setControl(transferVelocity);
        shooterMotorFollower.setControl(transferVelocity);
    }

    public void reverseIntake() {
        shooterMotor.setControl(reverseIntakeVelocity);
        shooterMotorFollower.setControl(reverseIntakeVelocity);
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
        } else /*if (Indexer.getInstance().hasCollected())*/{
            enable();
        }
    }

    // INPUTS

    void processInputs(ShooterIOInputs inputs) {
        setShooterAngle(inputs.angle);
    }

    void refreshSignals() {
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, shooterVelocity);
    }

    @Override
    public boolean logOutputs() {
        return OutputsConfiguration.SHOOTER;
    }
}
