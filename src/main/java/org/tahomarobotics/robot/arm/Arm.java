package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.arm.ArmConstants.*;

public class Arm extends SubsystemIF {
    private static final Arm INSTANCE = new Arm();

    private final TalonFX motor;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private Arm.State state = State.STOW;

    private final MotionMagicVoltage stowPose = new MotionMagicVoltage(STOW_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage transPose = new MotionMagicVoltage(TRANS_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage ampPose = new MotionMagicVoltage(AMP_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage trapPose = new MotionMagicVoltage(TRAP_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    public Arm() {
        RobustConfigurator configurator = new RobustConfigurator(logger);
        motor = new TalonFX(RobotMap.ARM_MOTOR);
        configurator.configureTalonFX(motor, ArmConstants.armMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, position, velocity);
        motor.optimizeBusUtilization();
    }

    public static Arm getInstance() {
        return INSTANCE;
    }

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(position.refresh(), velocity.refresh());
    }

    public double getVelocity() {
        return velocity.refresh().getValue();
    }

    public void stow() {
        motor.setControl(stowPose);
        state = State.STOW;
    }

    public void trans() {
        motor.setControl(transPose);
        state = State.TRANS;
    }

    public void amp() {
        motor.setControl(ampPose);
        state = State.AMP;
    }


    public void trap() {
        motor.setControl(trapPose);
        state = Arm.State.TRAP;
    }

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Position", getPosition());
        Logger.recordOutput("Arm/Velocity", getVelocity());

        Logger.recordOutput("Arm/State", state);
    }

    // STATES

    enum State {
        STOW,
        TRANS,
        AMP,
        TRAP
    }

}
