package org.tahomarobotics.robot.climbers.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;

import static org.tahomarobotics.robot.climbers.ClimberConstants.BOTTOM_POSITION;
import static org.tahomarobotics.robot.climbers.ClimberConstants.LADEN_SLOT;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        addCommands(
                new ClimbCommand(BOTTOM_POSITION, LADEN_SLOT),
                AmpArmCommands.AMP_ARM_TRAP.get(),
                Commands.runOnce(() -> AmpArm.getInstance().setRollerState(AmpArm.RollerState.TRAP))
        );
    }
}