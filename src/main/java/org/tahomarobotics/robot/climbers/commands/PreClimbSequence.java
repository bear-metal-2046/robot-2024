package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.Climbers;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

import static org.tahomarobotics.robot.climbers.ClimberConstants.TOP_POSITION;
import static org.tahomarobotics.robot.climbers.ClimberConstants.UNLADEN_SLOT;

/**
 * Passes through the note, puts the shooter up, then raises the hooks.
 */
public class PreClimbSequence extends SequentialCommandGroup {
    public PreClimbSequence() {
        Climbers climbers = Climbers.getInstance();

        addCommands(
                AmpArmCommands.ARM_TO_STOW.get(),
                AmpArmCommands.FEEDFORWARD.get(),
                Commands.runOnce(() -> Shooter.getInstance().setAngle(ShooterConstants.MAX_PIVOT_ANGLE)),
                new ClimbCommand(TOP_POSITION, UNLADEN_SLOT),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.READY))
            );
    }
}
