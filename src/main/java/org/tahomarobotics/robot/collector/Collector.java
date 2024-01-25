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
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
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

    private final MotionMagicVelocityVoltage collectVelocityControl = new MotionMagicVelocityVoltage(COLLECT_MAX_RPS).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage deployPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private boolean isCollecting;
    private boolean isStowed = true;

    private Collector() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
        deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
        collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

        configurator.configureTalonFX(deployLeft, deployMotorConfiguration);
        configurator.configureTalonFX(deployRight, deployMotorConfiguration.withMotorOutput(deployMotorConfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)));
        configurator.configureTalonFX(collectMotor, collectMotorConfiguration);

        deployPositionLeft = deployLeft.getPosition();
        deployPositionRight = deployRight.getPosition();
        deployVelocity = deployRight.getVelocity();
        collectVelocity = collectMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
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
    }

    @Override

    public SubsystemIF initialize() {

        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ZeroCollectorCommand())
                .ignoringDisable(true).schedule();

        return this;
    }

    public void zeroCollector() {
        deployLeft.setPosition(0);
        deployRight.setPosition(0);
    }

    public boolean isCollecting() {
        return isCollecting;
    }

    public boolean isStowed() {
        return isStowed;
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


    private void setDeployPosition(double position) {
        deployLeft.setControl(deployPositionControl.withPosition(position));
        deployRight.setControl(deployPositionControl.withPosition(position));
    }

    public void stowCollector() {
        setDeployPosition(STOW_POSITION);
        isStowed = true;
    }

    public void deployCollector() {
        setDeployPosition(COLLECT_POSITION);
        isStowed = false;
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

    public void toggleDeploy() {
        if (isStowed) {
            deployCollector();
        } else {
            stowCollector();
        }
    }
}