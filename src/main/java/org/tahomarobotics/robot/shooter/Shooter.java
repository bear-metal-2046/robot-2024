package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.shooter.ShooterConstants.shooterMotorConfiguration;


public class Shooter extends SubsystemIF {

    private static final Shooter INSTANCE = new Shooter();

    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final StatusSignal<Double> velocity;

    private final VelocityVoltage motorVelocity = new VelocityVoltage(0.0).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);

    private Shooter() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        topMotor = new TalonFX(RobotMap.TOP_SHOOTER_MOTOR);
        bottomMotor = new TalonFX(RobotMap.BOTTOM_SHOOTER_MOTOR);

        configurator.configureTalonFX(topMotor, shooterMotorConfiguration);
        configurator.configureTalonFX(bottomMotor, shooterMotorConfiguration);

        velocity = topMotor.getVelocity();

        bottomMotor.setControl(new Follower(topMotor.getDeviceID(), true));
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    // GETTERS

    public double getVelocity() {
        return velocity.refresh().getValue();
    }

    // SETTERS

    public void setVelocity(double speed) {
        topMotor.setControl(motorVelocity.withVelocity(speed));
    }

    public void stopShooter() {
        topMotor.set(0.0);
        bottomMotor.set(0.0);
    }
}
