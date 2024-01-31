package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimbCommand extends Command {
    private final Climbers climbers = Climbers.getInstance();

    private final double pos;

    private final Slot0Configs slotTuning;

    public ClimbCommand(double pos, Slot0Configs slotTuning) {
        this.pos = pos;
        this.slotTuning = slotTuning;
        addRequirements(this.climbers);
    }

    @Override
    public void initialize() {
        climbers.setSlotTuning(slotTuning);
        climbers.setTargetPos(pos);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return climbers.getLeftPos() - ClimberConstants.TOP_POSITION < ClimberConstants.POSITION_EPSILON
                && climbers.getRightPos() - ClimberConstants.TOP_POSITION < ClimberConstants.POSITION_EPSILON;
    }

    @Override
    public void end(boolean interrupted) {
        climbers.zeroPosition();
    }
}
