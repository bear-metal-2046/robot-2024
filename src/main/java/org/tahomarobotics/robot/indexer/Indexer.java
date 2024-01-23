package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
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

    private final MotionMagicVoltage intakePos = new MotionMagicVoltage(INTAKE_DISTANCE).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);
    private final MotionMagicVoltage transferPos = new MotionMagicVoltage(TRANSFER_DISTANCE).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVel = new MotionMagicVelocityVoltage(IDLE_VEL).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);

    private Indexer() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);
        beamBreak = new DigitalInput(RobotMap.BEAM_BREAK);

        configurator.configureTalonFX(motor, IndexerConstants.indexMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY, position, velocity);
        motor.optimizeBusUtilization();
    }

    public static Indexer getInstance() {
        return INSTANCE;
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

    public boolean isIndexing() {
        return state == State.INDEXING;
    }

    // SETTERS

    public boolean isBeamBroken() {
        return !beamBreak.get();
    }

    // STATE TRANSITIONS

    public void stop() {
        motor.stopMotor();

        state = State.DISABLED;
    }

    public void collect() {
        motor.setControl(idleVel);

        state = State.COLLECT;
    }

    public void index() {
        if (isIndexing() || collected) return;
        if (Math.abs(getPosition() - INTAKE_DISTANCE) < 0.01) {
            stop();
            motor.setPosition(0.0);

            collected = true;
        }

        motor.setPosition(0.0);
        motor.setControl(intakePos);

        state = State.INDEXING;
    }

    public void transferToShooter() {
        if (isIndexing() || !collected) return;
        if (Math.abs(getPosition() - TRANSFER_DISTANCE) < 0.01) {
            stop();
            motor.setPosition(0.0);

            collected = false;
        }

        motor.setControl(transferPos);
    }

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/State", state);
        Logger.recordOutput("Indexer/BeamBreak", isBeamBroken());
        Logger.recordOutput("Indexer/Collected", hasCollected());
    }

    // STATES

    enum State {
        COLLECT,
        INDEXING,
        DISABLED
    }
}
