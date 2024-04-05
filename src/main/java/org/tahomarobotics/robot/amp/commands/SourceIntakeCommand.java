package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.AmpArmConstants;

public class SourceIntakeCommand extends SequentialCommandGroup {
    public SourceIntakeCommand() {
        AmpArm ampArm = AmpArm.getInstance();

        addCommands(
            Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.COLLECTING)),
            Commands.waitSeconds(AmpArmConstants.SOURCE_COLLECT_ACCEL_TIME * 2),
            Commands.waitUntil(() -> (ampArm.getRollerCurrent() > AmpArmConstants.SOURCE_COLLECT_CURRENT)),
            Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.CENTERING)),
            Commands.waitUntil(ampArm::isRollerAtPosition).withTimeout(3),
            Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.COLLECTED))
        );
    }
}
