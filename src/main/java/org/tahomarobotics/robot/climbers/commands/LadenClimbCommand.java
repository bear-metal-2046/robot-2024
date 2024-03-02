package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;


public class LadenClimbCommand extends Command {
    private final Climbers climbers = Climbers.getInstance();

    private final double position;

    public LadenClimbCommand(double position) {
        this.position = position;

        addRequirements(climbers);
    }

    @Override
    public void initialize() {
        climbers.setPositionLaden(position);
    }

    @Override
    public boolean isFinished() {
        return climbers.getLeftPosition() - position < ClimberConstants.POSITION_EPSILON
                && climbers.getRightPosition() - position < ClimberConstants.POSITION_EPSILON;
    }
}
