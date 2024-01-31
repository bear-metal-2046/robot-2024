package org.tahomarobotics.robot.climbers;


import com.ctre.phoenix6.configs.Slot0Configs;
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

    public void zeroPosition() {
        leftClimber.zeroPosition();
        rightClimber.zeroPosition();
    }

    public void setSlotTuning(Slot0Configs slotConfig) {
        leftClimber.setSlotTuning(slotConfig);
        rightClimber.setSlotTuning(slotConfig);
    }

    public void setTargetPos(double targetPositionMeters) {
        leftClimber.setTargetPos(targetPositionMeters);
        rightClimber.setTargetPos(targetPositionMeters);
    }

    public double getLeftPos() {
        return leftClimber.getPosition();
    }

    public double getRightPos() {
        return rightClimber.getPosition();
    }

    public void runWithVelocity(double targetVelocityMps) {
        leftClimber.runWithVelocity(targetVelocityMps);
        rightClimber.runWithVelocity(targetVelocityMps);
    }
}

