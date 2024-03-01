package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;

public class ClimbingCancel extends SequentialCommandGroup {
    public ClimbingCancel() {
        Climbers climbers = Climbers.getInstance();

        addCommands(
                // Stop Hooks and let robot fall.
                new ClimbCommand(ClimberConstants.TOP_POSITION, ClimberConstants.LADEN_SLOT)
        );
    }
}
