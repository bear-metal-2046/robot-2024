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
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.indexer.IndexerConstants.*;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    private final TalonFX motor;
    private final DigitalInput beamBreak;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;

    private State state = State.DISABLED;
    private boolean collected = false;

    private final MotionMagicVoltage intakePos = new MotionMagicVoltage(INTAKE_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage transferPos = new MotionMagicVoltage(TRANSFER_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVel = new MotionMagicVelocityVoltage(CollectorConstants.COLLECT_MAX_RPS)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private Indexer() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);
        beamBreak = new DigitalInput(RobotMap.BEAM_BREAK);

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
        SmartDashboard.putData("Reset Indexer", runOnce(() -> collected = false));

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
        return collected;
    }

    // SETTERS

    public void setCollectState() {
        if (state == State.DISABLED) {
            motor.setControl(idleVel);
            state = State.COLLECT;
        }
    }

    public void setDisableState() {
        motor.stopMotor();
        state = State.DISABLED;
    }

    public void setTransferState() {
        if (state == State.DISABLED && collected) {
            motor.setPosition(0.0);
            motor.setControl(transferPos);
            state = State.TRANSFERRING;
        }
    }

    public boolean isBeamBroken() {
        return !beamBreak.get();
    }

    // STATE TRANSITIONS

    private void collectingState() {
        if (isBeamBroken()) {
            motor.setPosition(0.0);
            motor.setControl(intakePos);
            state = State.INDEXING;
        }
    }

    private void indexingState() {
        if (getPosition() >= INTAKE_DISTANCE - POSITION_TOLERANCE) {
            motor.stopMotor();
            collected = true;
            state = State.DISABLED;
        }
    }

    private void transferringState() {
        if (getPosition() >= TRANSFER_DISTANCE - POSITION_TOLERANCE) {
            motor.stopMotor();
            collected = false;
            state = State.DISABLED;
        }
    }

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/Position", getPosition());
        Logger.recordOutput("Indexer/Velocity", getVelocity());

        Logger.recordOutput("Indexer/State", state);
        Logger.recordOutput("Indexer/BeamBreak", isBeamBroken());
        Logger.recordOutput("Indexer/Collected", hasCollected());

        switch (state) {
            case DISABLED -> { /* Do Nothing */ }
            case COLLECT -> collectingState();
            case INDEXING -> indexingState();
            case TRANSFERRING -> transferringState();
        }
    }

    // STATES

    enum State {
        COLLECT,
        INDEXING,
        DISABLED,
        TRANSFERRING
    }
}