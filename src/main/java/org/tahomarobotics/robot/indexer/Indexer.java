package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.SysIdTest;

import static org.tahomarobotics.robot.indexer.IndexerConstants.*;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    private final TalonFX motor;
    private final DigitalInput beamBreak;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;

    private State state = State.DISABLED;
    private boolean collected = false;

    private final SysIdTest tester;

    private final MotionMagicVoltage intakePos = new MotionMagicVoltage(INTAKE_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage transferPos = new MotionMagicVoltage(TRANSFER_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVel = new MotionMagicVelocityVoltage(IDLE_VEL)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private Indexer() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);
        beamBreak = new DigitalInput(RobotMap.BEAM_BREAK);

        configurator.configureTalonFX(motor, IndexerConstants.indexMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY, position, velocity);
        motor.optimizeBusUtilization();

        tester = new SysIdTest(this, motor);
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
        if (isIndexing() && getPosition() >= INTAKE_DISTANCE) {
            stop();
            motor.setPosition(0.0);

            collected = true;
        }
        if (isIndexing() || collected) return;

        motor.setPosition(0.0);
        motor.setControl(intakePos);

        state = State.INDEXING;
    }

    public void transferToShooter() {
        if (isIndexing() && getPosition() >= TRANSFER_DISTANCE) {
            stop();
            motor.setPosition(0.0);

            collected = false;
        }
        if (isIndexing() || !collected) return;

        motor.setControl(transferPos);
    }

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/Position", getPosition());
        Logger.recordOutput("Indexer/Velocity", getVelocity());

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

    // SYSID

    public void registerSysIdCommands(CommandXboxController controller) {
        logger.warn("IN SYSID MODE");

        controller.povUp().whileTrue(tester.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.povDown().whileTrue(tester.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controller.povLeft().whileTrue(tester.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controller.povRight().whileTrue(tester.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        /* Manually stop logging with left bumper after we're done with the tests */
        /* This isn't necessary, but is convenient to reduce the size of the hoot file */
        controller.leftBumper().onTrue(new RunCommand(SignalLogger::stop));
    }
}
