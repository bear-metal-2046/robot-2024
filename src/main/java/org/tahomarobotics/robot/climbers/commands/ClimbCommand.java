package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;


public class ClimbCommand extends Command {
    private final Climbers climbers = Climbers.getInstance();

    private final double pos;

    private final int slot;

    public ClimbCommand(double pos, int slot) {
        this.pos = pos;
        this.slot = slot;
        addRequirements(this.climbers);
    }

    @Override
    public void initialize() {
        climbers.setTargetPos(pos, slot);
    }

    @Override
    public boolean isFinished() {
        return climbers.getLeftPos() - pos < ClimberConstants.POSITION_EPSILON
                && climbers.getRightPos() - pos < ClimberConstants.POSITION_EPSILON;
    }
}
