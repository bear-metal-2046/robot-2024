package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.SysIdTest;

import static org.tahomarobotics.robot.arm.ArmConstants.*;

public class Arm extends SubsystemIF {
    private static final Arm INSTANCE = new Arm();
    private final TalonFX motor;
    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private Arm.State state = State.STOW;

    private final MotionMagicVoltage armPose = new MotionMagicVoltage(0)
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

    // GETTERS

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(position.refresh(), velocity.refresh());
    }

    public double getVelocity() {
        return velocity.refresh().getValue();
    }

    public Arm.State getState() {
        return state;
    }

    // STATE TRANSITIONS

    public void stow() {
        setArmPose(STOW_POSE);
        state = State.STOW;
    }

    public void trans() {
        setArmPose(TRANS_POSE);
        state = State.TRANS;
    }

    public void amp() {
        setArmPose(AMP_POSE);
        state = State.AMP;
    }


    public void trap() {
        setArmPose(TRAP_POSE);
        state = Arm.State.TRAP;
    }

    private void setArmPose(double pose){
        double output = MathUtil.clamp(pose,ARM_MIN_POSE,ARM_MAX_POSE);
        motor.setControl(armPose.withPosition(output));
    }

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Position", getPosition()*360);
        Logger.recordOutput("Arm/Velocity", getVelocity());

        Logger.recordOutput("Arm/State", state);
    }

    // INITIALIZE
    @Override
    public SubsystemIF initialize(){
        motor.setPosition(0);
        return this;
    }

    // STATES

    public enum State {
        STOW,
        TRANS,
        AMP,
        TRAP
    }
}
