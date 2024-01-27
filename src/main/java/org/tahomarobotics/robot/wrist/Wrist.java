package org.tahomarobotics.robot.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.wrist.WristConstants.*;

public class Wrist extends SubsystemIF {
    private static final Wrist INSTANCE = new Wrist();
    private final TalonFX motor;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private Wrist.State state = Wrist.State.STOW;

    private final MotionMagicVoltage stowPose = new MotionMagicVoltage(STOW_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage transPose = new MotionMagicVoltage(TRANS_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage ampPose = new MotionMagicVoltage(AMP_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage trapPose = new MotionMagicVoltage(TRAP_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    public Wrist() {
        RobustConfigurator configurator = new RobustConfigurator(logger);
        motor = new TalonFX(RobotMap.WRIST_MOTOR);
        configurator.configureTalonFX(motor, WristConstants.wristMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, position, velocity);
        motor.optimizeBusUtilization();
    }

    public static Wrist getInstance() {
        return INSTANCE;
    }

    // GETTERS

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(position.refresh(), velocity.refresh());
    }

    public double getVelocity() {
        return velocity.refresh().getValue();
    }

    // STATE TRANSITIONS

    public void stow() {
        motor.setControl(stowPose);
        state = Wrist.State.STOW;
    }

    public void trans() {
        motor.setControl(transPose);
        state = Wrist.State.TRANS;
    }

    public void amp() {
        motor.setControl(ampPose);
        state = Wrist.State.AMP;
    }


    public void trap() {
        motor.setControl(trapPose);
        state = Wrist.State.TRAP;
    }

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Wrist/Position", getPosition()*360);
        Logger.recordOutput("Wrist/Velocity", getVelocity());

        Logger.recordOutput("Wrist/State", state);
    }

    // INITIALIZE
    @Override
    public SubsystemIF initialize(){
        motor.setPosition(0);
        return this;
    }

    // STATES

    enum State {
        STOW,
        TRANS,
        AMP,
        TRAP
    }
}
