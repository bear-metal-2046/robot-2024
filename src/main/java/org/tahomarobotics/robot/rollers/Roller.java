package org.tahomarobotics.robot.rollers;

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

import static org.tahomarobotics.robot.rollers.RollerConstants.*;

public class Roller extends SubsystemIF{
    private static final Roller INSTANCE = new Roller();

    private final TalonFX motor;
    //private final DigitalInput beamBreak;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;

    private Roller.State state = Roller.State.DISABLED;
    private boolean collected = false;

    private final MotionMagicVoltage intakePos = new MotionMagicVoltage(ROLLER_INTAKE_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage transferPos = new MotionMagicVoltage(ROLLER_TRANSFER_DISTANCE)
            .withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVel = new MotionMagicVelocityVoltage(CollectorConstants.COLLECT_MAX_RPS)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    private Roller() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        motor = new TalonFX(RobotMap.ROLLER_MOTOR);
        //beamBreak = new DigitalInput(RobotMap.BEAM_BREAK);

        configurator.configureTalonFX(motor, RollerConstants.rollerMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, position, velocity);
        motor.optimizeBusUtilization();
    }

    public static Roller getInstance() {
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
        return state == Roller.State.INDEXING;
    }

    // SETTERS

    //public boolean isBeamBroken() {
    //    return !beamBreak.get();
    //}

    // STATE TRANSITIONS

    public void stop() {
        motor.stopMotor();

        state = Roller.State.DISABLED;
    }

    public void collect() {
        motor.setControl(idleVel);

        state = Roller.State.COLLECT;
    }

    public void index() {

        if (isIndexing() && getPosition() >= ROLLER_INTAKE_DISTANCE - ROLLER_POSITION_TOLERANCE) {
            stop();
            motor.setPosition(0.0);

            collected = true;
        }
        if (isIndexing() || collected) return;

        motor.setPosition(0.0);
        motor.setControl(intakePos);

        state = Roller.State.INDEXING;
    }

    public void transferToShooter() {
        if (getPosition() >= ROLLER_TRANSFER_DISTANCE - ROLLER_POSITION_TOLERANCE) {
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
        Logger.recordOutput("Roller/Position", getPosition());
        Logger.recordOutput("Roller/Velocity", getVelocity());

        Logger.recordOutput("Roller/State", state);
        //Logger.recordOutput("Roller/BeamBreak", isBeamBroken());
        Logger.recordOutput("Roller/Collected", hasCollected());
    }

    // STATES

    enum State {
        COLLECT,
        INDEXING,
        DISABLED
    }
}
