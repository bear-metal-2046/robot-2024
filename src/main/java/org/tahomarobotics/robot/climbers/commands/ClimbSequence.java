package org.tahomarobotics.robot.climbers.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.Climbers;

import static org.tahomarobotics.robot.climbers.ClimberConstants.BOTTOM_POSITION;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        Climbers climbers = Climbers.getInstance();
        AmpArm ampArm = AmpArm.getInstance();

        addCommands(
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.CLIMBING)),
                new LadenClimbCommand(BOTTOM_POSITION),
                Commands.runOnce(ampArm::shiftNote),
                Commands.waitUntil(ampArm::isRollerAtPosition),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)),
                AmpArmCommands.AMP_ARM_TRAP.get(),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.TRAP)),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.CLIMBED))
        );
    }
}