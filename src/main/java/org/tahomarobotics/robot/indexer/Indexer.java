package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.OI;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.auto.Autonomous;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.indexer.IndexerConstants.*;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    private final TalonFX motor;
    private final DigitalInput collectorBeanBake;
    private final DigitalInput shooterBeanBake;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;

    private final StatusSignal<Double> current;

    private State state = State.DISABLED;
    private final MotionMagicVoltage transferPos = new MotionMagicVoltage(TRANSFER_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage collectVel = new MotionMagicVelocityVoltage(COLLECT_SPEED)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage ejectVel = new MotionMagicVelocityVoltage(-CollectorConstants.COLLECT_MAX_RPS)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage reverseIntakeVelocity = new MotionMagicVelocityVoltage(-ShooterConstants.TRANSFER_VELOCITY)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private double energyUsed = 0;

    private double totalCurrent = 0;

    private Indexer() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);
        collectorBeanBake = new DigitalInput(RobotMap.BEAM_BREAK_ONE);
        shooterBeanBake = new DigitalInput(RobotMap.BEAM_BREAK_TWO);

        configurator.configureTalonFX(motor, IndexerConstants.indexMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        current = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                position, velocity,
                current
        );
        motor.optimizeBusUtilization();
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putData("Reset Indexer", runOnce(() -> setState(State.DISABLED)));

        if (RobotState.isAutonomous())
            Indexer.getInstance().setState(Indexer.State.COLLECTED);

        return this;
    }

    // GETTERS

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(position, velocity);
    }

    public double getVelocity() {
        return velocity.getValue();
    }

    public boolean isCollected() {
        return state == State.COLLECTED;
    }

    public boolean isTransferring() {
        return state == State.TRANSFERRING;
    }

    public boolean isIndexing() {
        return state == State.INDEXING;
    }

    public State getState() {
        return state;
    }

    // SETTERS

    public boolean getCollectorBeanBake() {
        return !collectorBeanBake.get();
    }

    public boolean getShooterBeanBake() {
        return !shooterBeanBake.get();
    }

    public void setState(State state) {
        this.state = state;
    }

    public void zero() {
        motor.setPosition(0.0);
    }

    // STATE TRANSITIONS

    public void disable() {
        motor.stopMotor();
    }

    public void collect() {
        motor.setControl(collectVel);
    }

    public void index() {
        if (getShooterBeanBake()) {
            disable();
            zero();

            transitionToCollected();
        }
    }

    public void eject() {
        motor.setControl(ejectVel);
    }

    public void transfer() {
        if (getPosition() >= TRANSFER_DISTANCE - POSITION_TOLERANCE) {
            disable();
            zero();

            transitionToDisabled();
        }
    }

    // TRANSITIONS

    public void transitionToDisabled() {
        setState(State.DISABLED);
    }

    public void transitionToCollecting() {
        setState(State.COLLECTING);
    }

    public void transitionToIndexing() {
        zero();
        motor.setControl(collectVel);

        OI.getInstance().rumbleDrive();

        setState(State.INDEXING);
    }

    public void transitionToCollected() {
        setState(State.COLLECTED);
    }

    public void transitionToTransferring() {
        zero();
        motor.setControl(transferPos);

        setState(State.TRANSFERRING);
    }

    public void transitionToReverseIntaking() {
        zero();
        motor.setControl(reverseIntakeVelocity);

        setState(State.REVERSE_INDEXING);
    }

    public void transitionToEjecting() {
        setState(State.EJECTING);
    }
    
    
    //STATE MACHINE
    
    private void stateMachine() {
        switch (getState()) {
            case DISABLED -> {
                disable();

                if (Collector.getInstance().isCollecting() && !isCollected()) transitionToCollecting();
                if (Collector.getInstance().isEjecting()) transitionToEjecting();
            }
            case COLLECTING -> {
                collect();

                if (!Collector.getInstance().isCollecting()) transitionToDisabled();
                if (getCollectorBeanBake()) transitionToIndexing();
                if (Collector.getInstance().isEjecting()) transitionToEjecting();
            }
            case INDEXING -> {
                index();

                if (Collector.getInstance().isEjecting()) transitionToEjecting();
            }
            case EJECTING -> {
                eject();

                if (!Collector.getInstance().isEjecting()) transitionToDisabled();
            }
            case TRANSFERRING -> {
                transfer();

                if (!getCollectorBeanBake()) transitionToDisabled();
            }
            case COLLECTED -> {
                disable();

                if (RobotState.isAutonomous() && Autonomous.getInstance().isUsingLookupTable()) {
                    Double angle = Autonomous.getInstance().getSelectedAutoShotAngle();
                    if (angle != null) {
                        Shooter.getInstance().setAngle(angle);
                        return;
                    }
                }

                // Fix possible broken state
                if (!getCollectorBeanBake()) {
                    logger.error("Indexer in COLLECTED state with no note detected! Transitioning to DISABLED...");
                    transitionToDisabled();
                }
                if (Collector.getInstance().isEjecting()) transitionToEjecting();
            }
        }
    }
    

    // PERIODIC

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            position, velocity, current
        );

        double voltage = RobotController.getBatteryVoltage();
        totalCurrent = current.getValue();
        energyUsed += totalCurrent * voltage * Robot.defaultPeriodSecs;

        SafeAKitLogger.recordOutput("Indexer/Position", getPosition());
        SafeAKitLogger.recordOutput("Indexer/Velocity", getVelocity());

        SafeAKitLogger.recordOutput("Indexer/State", state);
        SafeAKitLogger.recordOutput("Indexer/BeamBreak One", getCollectorBeanBake());
        SafeAKitLogger.recordOutput("Indexer/BeamBreak Two", getShooterBeanBake());
        SafeAKitLogger.recordOutput("Indexer/Collected", isCollected());

        SafeAKitLogger.recordOutput("Indexer/Current", totalCurrent);
        SafeAKitLogger.recordOutput("Indexer/Energy", getEnergyUsed());

        stateMachine();

    }

    // STATES

    public enum State {
        DISABLED,
        COLLECTING,
        INDEXING,
        COLLECTED,
        EJECTING,
        TRANSFERRING,
        REVERSE_INDEXING
    }

    @Override
    public double getEnergyUsed() {
        return energyUsed / 1000d;
    }

    @Override
    public double getTotalCurrent() {
        return totalCurrent;
    }
}