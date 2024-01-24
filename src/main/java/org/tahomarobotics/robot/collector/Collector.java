package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.commands.DeployCommand;
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
    private final StatusSignal<Double> deployVoltage;
    private final StatusSignal<Double> followerVoltage;

    private final MotionMagicVelocityVoltage collectVelocityControl = new MotionMagicVelocityVoltage(COLLECT_MAX_RPS).withEnableFOC(RobotConfiguration.RIO_USING_PHOENIX_PRO);
    private final MotionMagicVoltage deployPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_USING_PHOENIX_PRO);

    private boolean isCollecting;

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
        deployVoltage = deployFollower.getSupplyVoltage();
        followerVoltage = deployFollower.getSupplyVoltage();

        ParentDevice.optimizeBusUtilizationForAll(deployMotor, deployFollower, collectMotor);

        BaseStatusSignal.setUpdateFrequencyForAll(100, deployPosition, collectVelocity, deployVoltage, followerVoltage);
    }

    @Override
    public void periodic() {
        //BaseStatusSignal.refreshAll(deployPosition, collectVelocity);
        double pos = getDeployPosition();
        SmartDashboard.putNumber("Collector Angle (Deg)", pos * 360);
        SmartDashboard.putNumber("Collector Angle (Rot)", pos);

        SmartDashboard.putNumber("Master Motor", deployVoltage.refresh().getValue());
        SmartDashboard.putNumber("Follower Motor", followerVoltage.refresh().getValue());
    }

    @Override

    public SubsystemIF initialize() {

        zeroCollector();

        SmartDashboard.putData(getDeployCommand(COLLECT_POSITION));
        return this;
    }

    public void zeroCollector() {
        deployMotor.setPosition(0);
    }

    public boolean isAtPosition(double desiredPosition) {
        return Math.abs(getDeployPosition() - desiredPosition) < EPSILON;
    }

    public boolean isCollecting() {
        return isCollecting;
    }

    public double getDeployPosition() {
        return deployPosition.refresh().getValue();
    }

    public double getCollectVelocity() {
        return collectVelocity.refresh().getValue() * COLLECT_GEAR_REDUCTION;
    }

    public void setDeployPosition(double position) {
        deployMotor.setControl(deployPositionControl.withPosition(position));
    }

    public void collect() {
        collectMotor.setControl(collectVelocityControl);
        isCollecting = true;
    }

    public void stopCollect() {
        collectMotor.stopMotor();
        isCollecting = false;
    }

    public Command getDeployCommand(double desiredPosition) {
        return new DeployCommand(this, desiredPosition);
    }
}
