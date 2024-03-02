package org.tahomarobotics.robot.climbers;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.climbers.commands.ClimbZeroCommand;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Climbers extends SubsystemIF {
    private final static Climbers INSTANCE = new Climbers();

    public static Climbers getInstance() {
        return INSTANCE;
    }

    private final Climber leftClimber, rightClimber;

    private ClimbState state = ClimbState.COCKED;

    private double energyUsed = 0;

    private Climbers() {
        leftClimber = new Climber(RobotMap.LEFT_CLIMB_MOTOR, "Left Climber", true);
        rightClimber = new Climber(RobotMap.RIGHT_CLIMB_MOTOR, "Right Climber", false);
    }

    // SETTERS

    public void zero() {
        leftClimber.zero();
        rightClimber.zero();
    }

    public void setPositionLaden(double position) {
        leftClimber.setPositionLaden(position);
        rightClimber.setPositionLaden(position);
    }

    public void setPositionUnladen(double position) {
        leftClimber.setPositionUnladen(position);
        rightClimber.setPositionUnladen(position);
    }

    public void setVoltageLeft(double voltage) {
        leftClimber.setVoltage(voltage);
    }

    public void setVoltageRight(double voltage) {
        rightClimber.setVoltage(voltage);
    }

    public void stop() {
        stopLeft();
        stopRight();
    }

    public void stopLeft() {
        leftClimber.stop();
    }

    public void stopRight() {
        rightClimber.stop();
    }

    // GETTERS

    public double getLeftPosition() {
        return leftClimber.getPosition();
    }

    public double getRightPosition() {
        return rightClimber.getPosition();
    }

    public double getLeftVelocity() {
        return leftClimber.getVelocity();
    }

    public double getRightVelocity() {
        return rightClimber.getVelocity();
    }

    @Override
    public double getEnergyUsed() {
        return energyUsed;
    }

    @Override
    public SubsystemIF initialize() {
        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ClimbZeroCommand())
                .ignoringDisable(true).schedule();

        return this;
    }

    @Override
    public void periodic() {
        double voltage = RobotController.getBatteryVoltage();
        double totalCurrent = leftClimber.getCurrent() + rightClimber.getCurrent();
        energyUsed += totalCurrent * voltage * Robot.defaultPeriodSecs;

        SafeAKitLogger.recordOutput("Climbers/Left Position", getLeftPosition());
        SafeAKitLogger.recordOutput("Climbers/Right Position", getRightPosition());
        SafeAKitLogger.recordOutput("Climbers/Left Current", leftClimber.getCurrent());
        SafeAKitLogger.recordOutput("Climbers/Right Current", rightClimber.getCurrent());
    }


    // STATE CONTROL

    public ClimbState getClimbState() {
        return state;
    }

    public void setClimbState(ClimbState state) {
        this.state = state;
    }

    // STATES

    public enum ClimbState {
        COCKED,
        READY,
        ENGAGED,
        CLIMBING,
        CLIMBED,
        BROKEN
    }
}

