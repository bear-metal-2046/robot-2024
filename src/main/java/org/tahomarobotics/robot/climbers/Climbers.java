package org.tahomarobotics.robot.climbers;


import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.climbers.commands.ClimbCommand;
import org.tahomarobotics.robot.climbers.commands.ClimbSequence;
import org.tahomarobotics.robot.climbers.commands.ClimbZeroCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.SysIdTest;

public class Climbers extends SubsystemIF {
    private final Logger logger = LoggerFactory.getLogger("Climbers");

    private final static Climbers INSTANCE = new Climbers();

    private final SysIdTest test;

    public static Climbers getInstance() {
        return INSTANCE;
    }

    private final Climber leftClimber;
    private final Climber rightClimber;

    private ClimbState state = ClimbState.COCKED;

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
        SmartDashboard.putData("Climb Sequence", Commands.deferredProxy(ClimbSequence::new));
        SmartDashboard.putData("Test Climb Command UP", new ClimbCommand(ClimberConstants.TOP_POSITION, ClimberConstants.UNLADEN_SLOT)); // TODO: Test Command, remove after testing
        SmartDashboard.putData("Test Climb Shooter Command UP", Commands.runOnce(() -> Shooter.getInstance().setAngle(ShooterConstants.MAX_PIVOT_ANGLE))); // TODO: Test Command, remove after testing
        SmartDashboard.putData("Test Climb Command DOWN", new ClimbCommand(ClimberConstants.BOTTOM_POSITION, ClimberConstants.LADEN_SLOT)); // TODO: Test Command, remove after testing
        SmartDashboard.putData("Test Climb Command MOSTLY-DOWN", new ClimbCommand(ClimberConstants.BOTTOM_POSITION + 0.05, ClimberConstants.LADEN_SLOT)); // TODO: Test Command, remove after testing
//        SmartDashboard.putData("Test Climb Command Down", new ClimbCommand(0, ClimberConstants.LADEN_SLOT));
        return this;
    }

    @Override
    public double getEnergyUsed() {
        // TODO
        return 0;
    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("Climb Left Position", getLeftPos());
//        SmartDashboard.putNumber("Climb Right Position", getRightPos());
        SafeAKitLogger.recordOutput("Climbers/Left Pos", getLeftPos());
        SafeAKitLogger.recordOutput("Climbers/Right Pos", getRightPos());
        SafeAKitLogger.recordOutput("Climbers/Left Voltage", leftClimber.voltage.refresh().getValueAsDouble());
        SafeAKitLogger.recordOutput("Climbers/Right Voltage", rightClimber.voltage.refresh().getValueAsDouble());
        SafeAKitLogger.recordOutput("Climbers/Left Current", leftClimber.current.refresh().getValueAsDouble());
        SafeAKitLogger.recordOutput("Climbers/Right Current", rightClimber.current.refresh().getValueAsDouble());
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

    public ClimbState getClimbState() {
        return state;
    }

    public void setClimbState(ClimbState state) {
        this.state = state;
    }

    public enum ClimbState {
        COCKED,
        READY,
        ENGAGED,
        CLIMBING,
        CLIMBED,
        BROKEN
    }
}

