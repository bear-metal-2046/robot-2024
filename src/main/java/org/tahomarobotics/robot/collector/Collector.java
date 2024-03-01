package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.commands.ZeroCollectorCommand;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.SysIdTest;
import org.tahomarobotics.robot.util.ToggledOutputs;

import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class Collector extends SubsystemIF implements ToggledOutputs {

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

    private final StatusSignal<Double> deployCurrentLeft;
    private final StatusSignal<Double> deployCurrentRight;
    private final StatusSignal<Double> collectCurrent;

    private final MotionMagicVelocityVoltage collectVelocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage deployPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private CollectionState collectionState = CollectionState.DISABLED;
    private DeploymentState deploymentState = DeploymentState.STOWED;

    private boolean isCollecting;
    private boolean isEjecting;
    private boolean isZeroed;

    private double energyUsed = 0;

    private Collector() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        deployLeft = new TalonFX(RobotMap.DEPLOY_MOTOR_LEFT);
        deployRight = new TalonFX(RobotMap.DEPLOY_MOTOR_RIGHT);
        collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

        configurator.configureTalonFX(deployLeft, deployMotorConfiguration, "deploy left motor");
        configurator.configureTalonFX(deployRight, deployMotorConfiguration.withMotorOutput(deployMotorConfiguration.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)), "deploy right motor");
        configurator.configureTalonFX(collectMotor, collectMotorConfiguration, "collect motor");

        deployPositionLeft = deployLeft.getPosition();
        deployPositionRight = deployRight.getPosition();
        deployVelocity = deployRight.getVelocity();
        collectVelocity = collectMotor.getVelocity();

        deployCurrentLeft = deployLeft.getSupplyCurrent();
        deployCurrentRight = deployRight.getSupplyCurrent();
        collectCurrent = collectMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                deployPositionLeft,
                deployPositionRight,
                deployVelocity,
                collectVelocity,
                deployCurrentLeft, deployCurrentRight, collectCurrent
        );

        ParentDevice.optimizeBusUtilizationForAll(deployLeft, deployRight, collectMotor);
    }

    // GETTERS

    private double getDeployPositionLeft() {
        return deployPositionLeft.getValue();
    }

    private double getDeployPositionRight() {
        return deployPositionRight.getValue();
    }

    public double getDeployVelocity() {
        return deployVelocity.getValue();
    }

    public double getCollectVelocity() {
        return collectVelocity.getValue();
    }


    // DEPLOYMENT CONTROL

    private void setDeployPosition(double position) {
        deployLeft.setControl(deployPositionControl.withPosition(position));
        deployRight.setControl(deployPositionControl.withPosition(position));
    }

    public void toggleDeploy() {
        if (deploymentState == DeploymentState.STOWED || deploymentState == DeploymentState.EJECT) {
            setDeployed();
            Shooter.getInstance().setAngle(ShooterConstants.SHOOTER_COLLECT_PIVOT_ANGLE);
        } else {
            setStowed();
        }
    }

    public boolean isDeployed() {
        return deploymentState == DeploymentState.DEPLOYED;
    }

    public void setDeployEject() {
        deploymentState = DeploymentState.EJECT;
        setDeployPosition(EJECT_POSITION);
    }

    public void setDeployed() {
        deploymentState = DeploymentState.DEPLOYED;
        setDeployPosition(COLLECT_POSITION);
    }

    public void setStowed() {
        deploymentState = DeploymentState.STOWED;
        setDeployPosition(STOW_POSITION);
    }


    // COLLECTOR CONTROL

    public void setIsCollecting(boolean collecting) {
        isCollecting = collecting;
    }

    public void setIsEjecting(boolean ejecting) {
        isEjecting = ejecting;
    }

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
        deployLeft.setPosition(ZERO_POSITION);
        deployRight.setPosition(ZERO_POSITION);

        isZeroed = true;
    }

    public boolean isZeroed() {
        return isZeroed;
    }


    //STATE MACHINES

    private void teleopStateMachine() {
        switch (getCollectionState()) {
            case DISABLED -> {
                stopCollect();

                if (isInEject()) toggleDeploy();

                if (isCollecting && !isStowed() && !Indexer.getInstance().hasCollected()) setCollectionState(Collector.CollectionState.COLLECTING);

                if (isEjecting) setCollectionState(Collector.CollectionState.EJECTING);
            }
            case COLLECTING -> {
                collect();

                if (Indexer.getInstance().hasCollected()) toggleDeploy();

                if ((!isCollecting && !Indexer.getInstance().isIndexing()) || isStowed()) setCollectionState(Collector.CollectionState.DISABLED);
            }
            case EJECTING -> {
                eject();

                if (!isStowed()) setDeployEject();

                if (isCollecting && !isStowed()) setCollectionState(Collector.CollectionState.COLLECTING);

                if (!isEjecting) setCollectionState(Collector.CollectionState.DISABLED);
            }
        }
    }

    private void autoStateMachine() {
        switch (getCollectionState()) {
            case DISABLED -> {
                stopCollect();

                if (isDeployed()) setCollectionState(Collector.CollectionState.COLLECTING);
            }
            case COLLECTING -> {
                collect();

                if (isStowed()) setCollectionState(Collector.CollectionState.DISABLED);
            }
        }
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(deployPositionLeft, deployPositionRight, collectVelocity, deployVelocity);
        double voltage = RobotController.getBatteryVoltage();
        energyUsed += (deployCurrentLeft.getValue() + deployCurrentRight.getValue() + collectCurrent.getValue()) * voltage * Robot.defaultPeriodSecs;

        recordOutput("Collector/Deploy State", deploymentState);
        recordOutput("Collector/Collection State", collectionState);

        recordOutput("Collector/Deploy Right Position", deployPositionRight.getValue());
        recordOutput("Collector/Deploy Left Position", deployPositionLeft.getValue());
        recordOutput("Collector/Deploy Velocity", deployVelocity.getValue());
        recordOutput("Collector/Collect Velocity", collectVelocity.getValue());
        recordOutput("Collector/Energy", getEnergyUsed());

        if (RobotState.isAutonomous()) {
            autoStateMachine();
        } else {
            teleopStateMachine();
        }

    }

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putBoolean("Outputs/Collector", OutputsConfiguration.COLLECTOR);

        Commands.waitUntil(RobotState::isEnabled)
                .andThen(Commands.waitSeconds(0.25))
                .andThen(new ZeroCollectorCommand())
                .ignoringDisable(true).schedule();

        Commands.waitUntil(() -> RobotState.isEnabled() && RobotState.isTeleop())
                .andThen(Commands.print("ZEROING COLLECTOR"))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(new ZeroCollectorCommand().onlyIf(() -> !isZeroed()))
                .ignoringDisable(true)
                .schedule();

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
        EJECT
    }

    @Override
    public boolean logOutputs() {
        return SmartDashboard.getBoolean("Outputs/Collector", OutputsConfiguration.COLLECTOR);
    }

    @Override
    public double getEnergyUsed() {
        return energyUsed / 1000d;
    }
}
