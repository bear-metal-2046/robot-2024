package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;


public class LadenClimbCommand extends Command {
    private static final Logger logger = LoggerFactory.getLogger(LadenClimbCommand.class);

    private final Climbers climbers = Climbers.getInstance();

    private final double position;

    public LadenClimbCommand(double position) {
        this.position = position;

        addRequirements(climbers);
    }

    @Override
    public void initialize() {
        logger.info("Laden Climb Initialized");
        climbers.setPositionLaden(position);
    }

    @Override
    public boolean isFinished() {
        return climbers.getLeftPosition() - position < ClimberConstants.POSITION_EPSILON
                && climbers.getRightPosition() - position < ClimberConstants.POSITION_EPSILON;
    }
}
