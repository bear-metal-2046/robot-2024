package org.tahomarobotics.robot.climbers;


import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Climbers extends SubsystemIF {
    private final Logger logger = LoggerFactory.getLogger("Climbers");

    private final static Climbers INSTANCE = new Climbers();

    public static Climbers getInstance() {
        return INSTANCE;
    }

    private final Climber leftClimber;
    private final Climber rightClimber;

    private Climbers() {
        leftClimber = new Climber(RobotMap.LEFT_CLIMB_MOTOR, "Left Climber");
        rightClimber = new Climber(RobotMap.RIGHT_CLIMB_MOTOR, "Right Climber");
    }

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putData("Climb Zero Command", new ClimbZeroCommand());
        SmartDashboard.putData("Climb Sequence", new ClimbSequence());
        SmartDashboard.putData("DeClimb Sequence", new DeclimbSequence());
        SmartDashboard.putData("Test Climb Command", new ClimbCommand(1, ClimberConstants.CLIMB_UNLADEN_SLOT)); // TODO: Test Command, remove after testing
        return this;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Left Position", getLeftPos());
        SmartDashboard.putNumber("Climb Right Position", getRightPos());
    }

    public void zeroToCurrentPosition() {
        leftClimber.zeroAtCurrentPosition();
        rightClimber.zeroAtCurrentPosition();
    }

    public void setSlotTuning(Slot0Configs slotConfig) {
        leftClimber.setSlotTuning(slotConfig);
        rightClimber.setSlotTuning(slotConfig);
    }

    public void setTargetPos(double targetPosition) {
        leftClimber.setTargetPos(targetPosition);
        rightClimber.setTargetPos(targetPosition);
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

    public void runWithVelocity(double targetVelocity) {
        leftClimber.runWithVelocity(targetVelocity);
        rightClimber.runWithVelocity(targetVelocity);
    }
}

