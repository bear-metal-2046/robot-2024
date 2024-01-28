package org.tahomarobotics.robot.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.SysIdTest;
import static org.tahomarobotics.robot.wrist.WristConstants.*;

public class Wrist extends SubsystemIF {
    private static final Wrist INSTANCE = new Wrist();
    private final TalonFX motor;

    private SysIdTest test;
    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private Wrist.State state = Wrist.State.STOW;

    private final MotionMagicVoltage wristPose = new MotionMagicVoltage(STOW_POSE)
            .withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    public Wrist() {
        RobustConfigurator configurator = new RobustConfigurator(logger);
        motor = new TalonFX(RobotMap.WRIST_MOTOR);
        configurator.configureTalonFX(motor, WristConstants.wristMotorConfiguration);

        position = motor.getPosition();
        velocity = motor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, position, velocity);
        motor.optimizeBusUtilization();

        test = new SysIdTest(this, motor);
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
        setWristPose(STOW_POSE);
        state = Wrist.State.STOW;
    }

    public void trans() {
        setWristPose(TRANS_POSE);
        state = Wrist.State.TRANS;
    }

    public void amp() {
        setWristPose(AMP_POSE);
        state = Wrist.State.AMP;
    }

    public void trap() {
        setWristPose(TRAP_POSE);
        state = Wrist.State.TRAP;
    }

    public double poseOutput;
    private void setWristPose(double pose){
        motor.setControl(wristPose.withPosition(pose));
        poseOutput = pose;
    }

    // PERIODIC

    @Override
    public void periodic() {
        Logger.recordOutput("Wrist/Position", getPosition()*360);
        Logger.recordOutput("Wrist/Velocity", getVelocity());
        Logger.recordOutput("Wrist/Desired Position", poseOutput * 360);
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

    public void registerSysIdCommands(CommandXboxController controller){
        controller.povUp().onTrue(test.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.povDown().onTrue(test.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controller.povLeft().onTrue(test.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controller.povRight().onTrue(test.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
}
