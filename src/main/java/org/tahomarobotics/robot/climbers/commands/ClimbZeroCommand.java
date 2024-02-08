package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;


public class ClimbZeroCommand extends Command {
    private final Climbers climbers = Climbers.getInstance();
    private final Timer timer = new Timer();

    public ClimbZeroCommand() {
        addRequirements(climbers);
    }

    @Override
    public void initialize() {
        timer.start();
        climbers.runWithVoltage(ClimberConstants.ZERO_VOLTAGE);
    }
    
    @Override
    public boolean isFinished() {
        return climbers.getLeftVel() < ClimberConstants.VELOCITY_EPSILON && climbers.getRightVel() < ClimberConstants.VELOCITY_EPSILON && timer.get() > 0.25;
    }

    @Override
    public void end(boolean interrupted) {
        climbers.stop();
        climbers.zeroToCurrentPosition();
    }
}
