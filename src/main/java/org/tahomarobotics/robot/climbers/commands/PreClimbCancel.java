package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;

public class PreClimbCancel extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(PreClimbCancel.class);

    public PreClimbCancel() {
        Climbers climbers = Climbers.getInstance();

        addCommands(
                Commands.runOnce(() -> logger.info("Canceled Pre-Climb")),
                // Lower Hooks
                new UnladenClimbCommand(ClimberConstants.BOTTOM_POSITION),
                AmpArmCommands.ARM_TO_STOW.get(),
                // Put arm down and feedback if the indexer is empty and the
                // amp arm has collected.
                AmpArmCommands.FEEDBACK.get(),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.COCKED))
        );
    }
}
