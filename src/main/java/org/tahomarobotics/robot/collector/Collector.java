package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class Collector extends SubsystemIF {

    private static final Collector INSTANCE = new Collector();

    public static Collector getInstance() {
        return INSTANCE;
    }
    private final TalonFX deployMotor;
    private final TalonFX deployFollower;
    private final TalonFX collectMotor;

    private final StatusSignal<Double> deployPosition;
    private final StatusSignal<Double> collectVelocity;

    private final MotionMagicVelocityVoltage collectVelocityControl = new MotionMagicVelocityVoltage(COLLECT_MAX_RPS);
    private final MotionMagicDutyCycle deployPositionControl = new MotionMagicDutyCycle(0.0).withEnableFOC(true);

    private final RobustConfigurator configurator;

    public Collector() {
        configurator = new RobustConfigurator(logger);

        deployMotor = new TalonFX(RobotMap.DEPLOY_COLLECTOR_MOTOR);
        deployFollower = new TalonFX(RobotMap.DEPLOY_COLLECTOR_MOTOR_FOLLOWER);
        collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

        configurator.configureTalonFX(deployMotor, deployMotorConfiguration, deployFollower, true);
        configurator.configureTalonFX(collectMotor, collectMotorConfiguration);

        deployPosition = deployMotor.getPosition();
        collectVelocity = collectMotor.getVelocity();

        ParentDevice.optimizeBusUtilizationForAll(deployMotor, deployFollower, collectMotor);

    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(deployPosition, collectVelocity);
    }

    @Override
    public SubsystemIF initialize() {
        deployMotor.setPosition(0);
        return this;
    }

    public void zeroCollector() {
        deployMotor.setPosition(0);
    }

    public boolean isAtPosition(double desiredPosition) {
        return Math.abs(getDeployPosition() - desiredPosition) < EPSILON;
    }

    public double getDeployPosition() {
        return deployPosition.refresh().getValue() * DEPLOY_GEAR_REDUCTION;
    }

    public double getCollectVelocity() {
        return collectVelocity.refresh().getValue() * COLLECT_GEAR_REDUCTION;
    }

    public void setDeployPosition(double position) {
        deployMotor.setControl(deployPositionControl.withPosition(position));
    }

    public void collect() {
        collectMotor.setControl(collectVelocityControl);
    }

    public void stopCollect() {
        collectMotor.stopMotor();
    }

    public Command getDeployCommand(double desiredPosition) {
        return new DeployCommand(this, desiredPosition);
    }
}
