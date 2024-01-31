package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.indexer.IndexerConstants.POSITION_TOLERANCE;
import static org.tahomarobotics.robot.indexer.IndexerConstants.TRANSFER_DISTANCE;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    private final TalonFX motor;
    private final DigitalInput beanBakeOne;
    private final DigitalInput beanBakeTwo;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;

    private State state = State.DISABLED;
    private final MotionMagicVoltage transferPos = new MotionMagicVoltage(TRANSFER_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage collectVel = new MotionMagicVelocityVoltage(CollectorConstants.COLLECT_MAX_RPS)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage ejectVel = new MotionMagicVelocityVoltage(-CollectorConstants.COLLECT_MAX_RPS)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage reverseIntakeVelocity = new MotionMagicVelocityVoltage(-ShooterConstants.TRANSFER_VELOCITY)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private Indexer() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);
        beanBakeOne = new DigitalInput(RobotMap.BEAM_BREAK_ONE);
        beanBakeTwo = new DigitalInput(RobotMap.BEAM_BREAK_TWO);

        configurator.configureTalonFX(motor, IndexerConstants.indexMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, position, velocity);
        motor.optimizeBusUtilization();
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putData("Reset Indexer", runOnce(() -> setState(State.DISABLED)));

        return this;
    }

    // GETTERS

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(position.refresh(), velocity.refresh());
    }

    public double getVelocity() {
        return velocity.refresh().getValue();
    }

    public boolean hasCollected() {
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

    public boolean isBeanBakeOne() {
        return !beanBakeOne.get();
    }

    public boolean isBeanBakeTwo() {
        return !beanBakeTwo.get();
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
        if (isBeanBakeTwo()) {
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

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/Position", getPosition());
        Logger.recordOutput("Indexer/Velocity", getVelocity());

        Logger.recordOutput("Indexer/State", state);
        Logger.recordOutput("Indexer/BeamBreakOne", isBeanBakeOne());
        Logger.recordOutput("Indexer/BeamBreakTwo", isBeanBakeTwo());
        Logger.recordOutput("Indexer/Collected", hasCollected());
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
}