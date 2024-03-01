package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.chassis.commands.DriveForwardCommand;
import org.tahomarobotics.robot.climbers.Climbers;

public class EngagedCancel extends SequentialCommandGroup {
    public EngagedCancel() {
        Climbers climbers = Climbers.getInstance();

        addCommands(
                new DriveForwardCommand(-.15, -1), // TODO: I have no idea if this command works lol.
                AmpArmCommands.ARM_TO_STOW.get().andThen(AmpArmCommands.FEEDBACK.get()),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.READY))
        );
    }
}
