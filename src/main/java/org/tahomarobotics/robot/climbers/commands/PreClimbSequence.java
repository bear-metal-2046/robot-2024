package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.Climbers;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

import static org.tahomarobotics.robot.climbers.ClimberConstants.TOP_POSITION;

/**
 * Passes through the note, puts the shooter up, then raises the hooks.
 */
public class PreClimbSequence extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(PreClimbSequence.class);

    public PreClimbSequence() {
        Climbers climbers = Climbers.getInstance();

        climbers.setTrapping(Indexer.getInstance().isCollected());

        addCommands(
                Commands.runOnce(() -> logger.info("Pre-Climb Sequence Started")),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.READY)),
                AmpArmCommands.ARM_TO_STOW.get()
        );

        if (climbers.isTrapping())
            addCommands(AmpArmCommands.FEEDFORWARD.get());

        addCommands(
                Commands.runOnce(() -> Shooter.getInstance().setAngle(ShooterConstants.MAX_PIVOT_ANGLE)),
                Commands.waitUntil(() -> Shooter.getInstance().isAtAngle()),
                new UnladenClimbCommand(TOP_POSITION),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.READY))
        );
    }
}
