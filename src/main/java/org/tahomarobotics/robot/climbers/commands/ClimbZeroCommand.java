package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;


public class ClimbZeroCommand extends Command {

    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ClimbZeroCommand.class);

    private final Climbers climbers = Climbers.getInstance();
    private final Timer timer = new Timer();

    private boolean left, right;

    public ClimbZeroCommand() {
        addRequirements(climbers);
    }

    @Override
    public void initialize() {
        logger.info("Zeroing Climbers");
        timer.restart();
    }

    @Override
    public void execute() {
        if (left || Math.abs(climbers.getLeftVelocity()) < ClimberConstants.VELOCITY_EPSILON && timer.hasElapsed(0.5)) {
            climbers.stopLeft();
            left = true;
        } else {
            climbers.setVoltageLeft(ClimberConstants.ZERO_VOLTAGE);
        }
        if (right || Math.abs(climbers.getRightVelocity()) < ClimberConstants.VELOCITY_EPSILON && timer.hasElapsed(0.5)) {
            climbers.stopRight();
            right = true;
        } else {
            climbers.setVoltageRight(ClimberConstants.ZERO_VOLTAGE);
        }
    }

    @Override
    public boolean isFinished() {
        return left && right;
    }

    @Override
    public void end(boolean interrupted) {
        climbers.zero();
        logger.info("ZEROED CLIMBERS");
    }
}
