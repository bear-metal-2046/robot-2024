package org.tahomarobotics.robot.climbers;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.climbers.commands.ClimbZeroCommand;
import org.tahomarobotics.robot.identity.RobotID;
import org.tahomarobotics.robot.identity.RobotIdentity;
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

    private double totalCurrent = 0;

    private boolean trapping = false;

    private final boolean enabled = RobotIdentity.robotID != RobotID.PLAYBEAR_CARTI;

    private Climbers() {
        leftClimber = enabled ? new Climber(RobotMap.LEFT_CLIMB_MOTOR, "Left Climber", true) : null;
        rightClimber = enabled ? new Climber(RobotMap.RIGHT_CLIMB_MOTOR, "Right Climber", false) : null;
        
        setTrapping(false);
    }

    // SETTERS

    public void zero() {
        if (!enabled) return;

        leftClimber.zero();
        rightClimber.zero();
    }

    public void setPositionLaden(double position) {
        if (!enabled) return;

        leftClimber.setPositionLaden(position);
        rightClimber.setPositionLaden(position);
    }

    public void setPositionUnladen(double position) {
        if (!enabled) return;

        leftClimber.setPositionUnladen(position);
        rightClimber.setPositionUnladen(position);
    }

    public void setVoltageLeft(double voltage) {
        if (!enabled) return;

        leftClimber.setVoltage(voltage);
    }

    public void setVoltageRight(double voltage) {
        if (!enabled) return;

        rightClimber.setVoltage(voltage);
    }

    public void stop() {
        if (!enabled) return;

        stopLeft();
        stopRight();
    }

    public void stopLeft() {
        if (!enabled) return;

        leftClimber.stop();
    }

    public void stopRight() {
        if (!enabled) return;

        rightClimber.stop();
    }

    // GETTERS

    public double getLeftPosition() {
        return enabled ? leftClimber.getPosition() : 0;
    }

    public double getRightPosition() {
        return enabled ? rightClimber.getPosition() : 0;
    }

    public double getLeftVelocity() {
        return enabled ? leftClimber.getVelocity() : 0;
    }

    public double getRightVelocity() {
        return enabled ? rightClimber.getVelocity() : 0;
    }

    public double getLeftCurrent() { return enabled ? leftClimber.getCurrent() : 0; }
    public double getRightCurrent() { return enabled ? rightClimber.getCurrent() : 0; }

    @Override
    public double getEnergyUsed() {
        return energyUsed;
    }

    @Override
    public double getTotalCurrent() {
        return totalCurrent;
    }

    public boolean isTrapping() {
        return trapping;
    }

    public void setTrapping(boolean trapping) {
        SafeAKitLogger.recordOutput("Climbers/Is Trapping?", trapping);
        this.trapping = trapping;
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
        if (!enabled) return;

        double voltage = RobotController.getBatteryVoltage();
        totalCurrent = Math.abs(leftClimber.getCurrent()) + Math.abs(rightClimber.getCurrent());
        energyUsed += totalCurrent * voltage * Robot.defaultPeriodSecs;

        SafeAKitLogger.recordOutput("Climbers/Left Position", getLeftPosition());
        SafeAKitLogger.recordOutput("Climbers/Right Position", getRightPosition());
        SafeAKitLogger.recordOutput("MotorCurrents/Climbers Left", leftClimber.getCurrent());
        SafeAKitLogger.recordOutput("MotorCurrents/Climbers Right", rightClimber.getCurrent());
        SafeAKitLogger.recordOutput("Climbers/Total Current", totalCurrent);
        SafeAKitLogger.recordOutput("Climbers/Energy", energyUsed);
        SafeAKitLogger.recordOutput("Climbers/Climb State", getClimbState());
    }


    // STATE CONTROL

    public ClimbState getClimbState() {
        return state;
    }

    public void setClimbState(ClimbState state) {
        logger.info("Set Climb State To: " + state.name());
        this.state = state;
    }

    // STATES

    public enum ClimbState {
        COCKED,
        PRE_CLIMBING,
        READY,
        ENGAGING,
        ENGAGED,
        CLIMBING,
        CLIMBED
    }
}

