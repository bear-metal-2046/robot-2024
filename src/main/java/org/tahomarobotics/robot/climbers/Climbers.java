package org.tahomarobotics.robot.climbers;


import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.climbers.commands.ClimbCommand;
import org.tahomarobotics.robot.climbers.commands.ClimbSequence;
import org.tahomarobotics.robot.climbers.commands.ClimbZeroCommand;
import org.tahomarobotics.robot.climbers.commands.DeclimbSequence;
import org.tahomarobotics.robot.collector.commands.ZeroCollectorCommand;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.SysIdTest;
import org.tahomarobotics.robot.util.ToggledOutputs;

public class Climbers extends SubsystemIF implements ToggledOutputs {
    private final Logger logger = LoggerFactory.getLogger("Climbers");

    private final static Climbers INSTANCE = new Climbers();

    private final SysIdTest test;

    public static Climbers getInstance() {
        return INSTANCE;
    }

    private final Climber leftClimber;
    private final Climber rightClimber;

    private Climbers() {
        leftClimber = new Climber(RobotMap.LEFT_CLIMB_MOTOR, "Left Climber", true);
        rightClimber = new Climber(RobotMap.RIGHT_CLIMB_MOTOR, "Right Climber", false);
        test = new SysIdTest(this, leftClimber.getMotor(), rightClimber.getMotor());
    }

    public void zeroToCurrentPosition() {
        leftClimber.zeroAtCurrentPosition();
        rightClimber.zeroAtCurrentPosition();
    }

    public void setTargetPos(double targetPosition, int slot) {
        leftClimber.setTargetPos(targetPosition, slot);
        rightClimber.setTargetPos(targetPosition, slot);
    }

    public double getLeftPos() {
        return leftClimber.getPosition();
    }

    public double getRightPos() {
        return rightClimber.getPosition();
    }

    public double getLeftVel() {
        return leftClimber.getVelocity();
    }

    public double getRightVel() {
        return rightClimber.getVelocity();
    }

    public void stop() {
        leftClimber.stop();
        rightClimber.stop();
    }

    public void runWithVoltage(double targetVoltage) {
        leftClimber.runWithVoltage(targetVoltage);
        rightClimber.runWithVoltage(targetVoltage);
    }

    public void registerSysIdCommands(CommandXboxController controller) {
        controller.povUp().whileTrue(test.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.povDown().whileTrue(test.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controller.povLeft().whileTrue(test.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controller.povRight().whileTrue(test.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public SubsystemIF initialize() {
        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ClimbZeroCommand())
                .ignoringDisable(true).schedule();

        SmartDashboard.putData("Climb Zero Command", new ClimbZeroCommand());
        SmartDashboard.putData("Climb Sequence", new ClimbSequence());
        SmartDashboard.putData("DeClimb Sequence", new DeclimbSequence());
        SmartDashboard.putData("Test Climb Command", new ClimbCommand(1, ClimberConstants.UNLADEN_SLOT)); // TODO: Test Command, remove after testing
        SmartDashboard.putData("Test Climb Command Down", new ClimbCommand(0, ClimberConstants.LADEN_SLOT));
        return this;
    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Climb Left Position", getLeftPos());
//        SmartDashboard.putNumber("Climb Right Position", getRightPos());
        recordOutput("Climbers/Left Pos", getLeftPos());
        recordOutput("Climbers/Right Pos", getRightPos());
    }

    public void runLeftWithVoltage(double targetVoltage) {
        leftClimber.runWithVoltage(targetVoltage);
    }

    public void runRightWithVoltage(double targetVoltage) {
        rightClimber.runWithVoltage(targetVoltage);
    }

    public void stopLeft() {
        leftClimber.stop();
    }

    public void stopRight() {
        rightClimber.stop();
    }

    @Override
    public boolean logOutputs() {
        return OutputsConfiguration.CLIMBER;
    }
}

