package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.apache.logging.log4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;
import org.tahomarobotics.robot.collector.commands.ZeroCollectorCommand;


public class ClimbZeroCommand extends Command {

    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ClimbZeroCommand.class);

    private final Climbers climbers = Climbers.getInstance();
    private final Timer timer = new Timer();

    public ClimbZeroCommand() {
        addRequirements(climbers);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        climbers.runWithVoltage(ClimberConstants.ZERO_VOLTAGE);
        logger.info(climbers.getLeftVel() + "   " + climbers.getRightVel());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climbers.getLeftVel()) < ClimberConstants.VELOCITY_EPSILON && Math.abs(climbers.getRightVel()) < ClimberConstants.VELOCITY_EPSILON && timer.get() > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        climbers.stop();
        climbers.zeroToCurrentPosition();
        logger.info("ZEROED CLIMBERS");
    }
}
