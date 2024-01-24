package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.commands.DeployCommand;
import org.tahomarobotics.robot.collector.commands.ZeroCollectorCommand;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class Collector extends SubsystemIF {

    private static final Collector INSTANCE = new Collector();

    public static Collector getInstance() {
        return INSTANCE;
    }

    private final TalonFX deployLeft;
    private final TalonFX deployRight;
    private final TalonFX collectMotor;

    private final StatusSignal<Double> deployPositionLeft;
    private final StatusSignal<Double> deployPositionRight;
    private final StatusSignal<Double> deployVelocity;
    private final StatusSignal<Double> collectVelocity;

    private final MotionMagicVelocityVoltage collectVelocityControl = new MotionMagicVelocityVoltage(COLLECT_MAX_RPS).withEnableFOC(RobotConfiguration.RIO_USING_PHOENIX_PRO);
    private final MotionMagicVoltage deployPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_USING_PHOENIX_PRO);

    private boolean isCollecting;

    private final RobustConfigurator configurator;

    private Collector() {
        configurator = new RobustConfigurator(logger);

        deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
        deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
        collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

        configurator.configureTalonFX(deployLeft, deployMotorConfiguration);
        configurator.configureTalonFX(deployLeft, deployMotorConfiguration.withMotorOutput(deployMotorConfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)));
        configurator.configureTalonFX(collectMotor, collectMotorConfiguration);

        deployPositionLeft = deployLeft.getPosition();
        deployPositionRight = deployRight.getPosition();
        deployVelocity = deployRight.getVelocity();
        collectVelocity = collectMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
                deployPositionLeft,
                deployPositionRight,
                deployVelocity,
                collectVelocity
        );

        ParentDevice.optimizeBusUtilizationForAll(deployLeft, deployRight, collectMotor);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(deployPositionLeft, collectVelocity);

        SmartDashboard.putNumber("Collector Angle LEFT (Deg)", getDeployPositionLeft() * 360);
        SmartDashboard.putNumber("Collector Angle RIGHT (Deg)", getDeployPositionLeft() * 360);
    }

    @Override

    public SubsystemIF initialize() {

        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ZeroCollectorCommand())
                .ignoringDisable(true).schedule();

        SmartDashboard.putData(getDeployCommand(COLLECT_POSITION, "down"));
        SmartDashboard.putData(getDeployCommand(STOW_POSITION, "up"));
        return this;
    }

    public void zeroCollector() {
        deployLeft.setPosition(0);
    }

    public boolean isAtPosition(double desiredPosition) {
        return Math.abs(getDeployPositionLeft() - desiredPosition) < EPSILON
                && Math.abs(getDeployPositionRight() - desiredPosition) < EPSILON;
    }

    public boolean isCollecting() {
        return isCollecting;
    }


    private double getDeployPositionLeft() {
        return deployPositionLeft.refresh().getValue();
    }

    private double getDeployPositionRight() {
        return deployPositionRight.refresh().getValue();
    }

    public double getDeployVelocity() {
        return deployVelocity.refresh().getValue();
    }

    public double getCollectVelocity() {
        return collectVelocity.refresh().getValue();
    }


    public void setDeployPosition(double position) {
        deployLeft.setControl(deployPositionControl.withPosition(position));
        deployRight.setControl(deployPositionControl.withPosition(position));
    }

    public void setVoltage(double voltage) {
        deployLeft.setControl(new VoltageOut(voltage));
        deployRight.setControl(new VoltageOut(voltage));
    }

    public void collect() {
        collectMotor.setControl(collectVelocityControl);
        isCollecting = true;
    }

    public void stopCollect() {
        collectMotor.stopMotor();
        isCollecting = false;
    }

    public void stopDeploy() {
        deployLeft.stopMotor();
        deployRight.stopMotor();
    }

    public Command getDeployCommand(double desiredPosition, String name) {
        var command = new DeployCommand(this, desiredPosition);

        command.setName(name);

        return command;
    }
}
