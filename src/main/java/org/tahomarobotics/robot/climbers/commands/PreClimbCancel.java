package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.ClimberConstants;

public class PreClimbCancel extends SequentialCommandGroup {
    public PreClimbCancel() {
        addCommands(
                // Lower Hooks
                new ClimbCommand(ClimberConstants.BOTTOM_POSITION, ClimberConstants.UNLADEN_SLOT),
                // Put arm down and feedback if the indexer is empty and the
                // amp arm has collected.
                AmpArmCommands.FEEDBACK.get()
        );
    }
}
