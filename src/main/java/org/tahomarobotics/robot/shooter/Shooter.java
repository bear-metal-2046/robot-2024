package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.BetterMath;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;


public class Shooter extends SubsystemIF {

    private static final Shooter INSTANCE = new Shooter();

    private final TalonFX shooterMotor;
    private final TalonFX pivotMotor;

    private final StatusSignal<Double> shooterVelocity;
    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotVelocity;

    private final MotionMagicVoltage pivotPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage motorVelocity = new MotionMagicVelocityVoltage(SHOOTER_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private double angle = 0.0;

    private Shooter() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
        shooterMotor = new TalonFX(RobotMap.TOP_SHOOTER_MOTOR);
        var shooterMotorFollower = new TalonFX(RobotMap.BOTTOM_SHOOTER_MOTOR);

        configurator.configureTalonFX(pivotMotor, pivotMotorConfiguration);
        configurator.configureTalonFX(shooterMotor, shooterMotorConfiguration, shooterMotorFollower, false);

        shooterVelocity = shooterMotor.getVelocity();
        pivotPosition = pivotMotor.getPosition();
        pivotVelocity = pivotMotor.getVelocity();
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        pivotMotor.setPosition(0.0);

        return this;
    }

    // GETTERS

    public double getShooterVelocity() {
        return shooterVelocity.refresh().getValue();
    }

    public double getPivotPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(pivotPosition.refresh(), pivotVelocity.refresh());
    }

    public double getPivotVelocity() {
        return pivotVelocity.refresh().getValue();
    }

    public boolean isSpinningAtVelocity() {
        return Math.abs(SHOOTER_SPEED - getShooterVelocity()) < SHOOTER_SPEED_TOLERANCE;
    }

    // SETTERS

    public void setShooterAngle(double _angle) {
        angle = BetterMath.clamp(_angle, 0, MAX_PIVOT_ANGLE);

        pivotMotor.setControl(pivotPositionControl.withPosition(angle));
    }

    public void setAngleToSpeaker() {
        Translation2d target = SPEAKER_TARGET_POSITION.get();
        double distance = Chassis.getInstance().getPose().getTranslation().getDistance(target);

        angle = Math.atan2(SPEAKER_HEIGHT_DIFF, distance) / (2 * Math.PI);

        if (angle < 0.0 || angle > MAX_PIVOT_ANGLE) {
            logger.warn("Out of range!");
        }

        setShooterAngle(angle);
    }

    // STATES

    public void enable() {
        shooterMotor.setControl(motorVelocity);
    }

    public void disable() {
        shooterMotor.stopMotor();
    }

    // onInit

    @Override
    public void onDisabledInit() {
        disable();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Angle (Degrees)", getPivotPosition() * 360);
    }
}
