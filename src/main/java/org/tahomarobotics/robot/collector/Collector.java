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
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.commands.ZeroCollectorCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
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

    private final MotionMagicVelocityVoltage collectVelocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage deployPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deploymentState = DeploymentState.STOWED;

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


    // GETTERS

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


    // DEPLOYMENT CONTROL

    private void setDeployPosition(double position) {
        deployLeft.setControl(deployPositionControl.withPosition(position));
        deployRight.setControl(deployPositionControl.withPosition(position));
    }

    public void toggleDeploy() {
        if (deploymentState == DeploymentState.STOWED || deploymentState == DeploymentState.EJECT) {
            deploymentState = DeploymentState.DEPLOYED;
            setDeployPosition(COLLECT_POSITION);
            Shooter.getInstance().setAngle(ShooterConstants.SHOOTER_COLLECT_PIVOT_ANGLE);
        } else {
            deploymentState = DeploymentState.STOWED;
            setDeployPosition(STOW_POSITION);
        }
    }

    public void setDeployEject() {
        deploymentState = DeploymentState.EJECT;
        setDeployPosition(EJECT_POSITION);
    }


    // COLLECTOR CONTROL

    public void collect() {
        collectMotor.setControl(collectVelocityControl.withVelocity(COLLECT_MAX_RPS));
    }

    public void eject() {
        collectMotor.setControl(collectVelocityControl.withVelocity(-COLLECT_MAX_RPS));
    }

    public void stopCollect() {
        collectMotor.stopMotor();
    }


    // STATE CONTROL

    public void setCollectionState(CollectionState state) {
        collectionState = state;
    }

    public CollectionState getCollectionState() {
        return collectionState;
    }

    public boolean isCollecting() {
        return collectionState == CollectionState.COLLECTING;
    }
    public boolean isEjecting() {
        return collectionState == CollectionState.EJECTING;
    }
    public boolean isStowed() {
        return deploymentState == DeploymentState.STOWED;
    }
    public boolean isInEject() {
        return deploymentState == DeploymentState.EJECT;
    }


    // ZEROING

    public void setVoltage(double voltage) {
        deployLeft.setControl(new VoltageOut(voltage));
        deployRight.setControl(new VoltageOut(voltage));
    }

    public void stopDeploy() {
        deployLeft.stopMotor();
        deployRight.stopMotor();
    }

    public void zeroCollector() {
        deployLeft.setPosition(0);
        deployRight.setPosition(0);
    }


    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(deployPositionLeft, deployPositionRight, collectVelocity);

        Logger.recordOutput("Collector/Deploy State", deploymentState);
        Logger.recordOutput("Collector/Collection State", collectionState);

        Logger.recordOutput("Collector/Deploy Right Position", deployPositionRight.getValue());
        Logger.recordOutput("Collector/Deploy Left Position", deployPositionLeft.getValue());
        Logger.recordOutput("Collector/Deploy Velocity", deployVelocity.getValue());
        Logger.recordOutput("Collector/Collect Velocity", collectVelocity.getValue());
    }

    @Override
    public SubsystemIF initialize() {

        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ZeroCollectorCommand())
                .ignoringDisable(true).schedule();

        return this;
    }

    // STATES

    public enum CollectionState {
        COLLECTING,
        DISABLED,
        EJECTING
    }

    public enum DeploymentState {
        DEPLOYED,
        STOWED,
        EJECT;
    }
}
