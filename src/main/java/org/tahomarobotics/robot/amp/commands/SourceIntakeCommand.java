package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.AmpArm;

public class SourceIntakeCommand extends SequentialCommandGroup {
    public SourceIntakeCommand() {
        AmpArm ampArm = AmpArm.getInstance();

        addCommands(
            Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.PASSING)),
            Commands.sequence(
                Commands.runOnce(ampArm::sourceIntake),
                Commands.waitUntil(ampArm::isRollerAtPosition),
                    Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.COLLECTED))
            ).onlyIf(() -> ampArm.getRollerCurrent() > 5.0)
        );
    }
}
