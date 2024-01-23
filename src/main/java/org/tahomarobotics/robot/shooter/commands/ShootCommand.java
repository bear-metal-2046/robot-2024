package org.tahomarobotics.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;

public class ShootCommand extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(ShootCommand.class);

    private final Indexer indexer = Indexer.getInstance();

    public ShootCommand() {
        Shooter shooter = Shooter.getInstance();

        addCommands(
                Commands.sequence(
//                        Commands.run(shooter::setAngleToSpeaker, shooter),
                        Commands.run(() -> shooter.setShooterAngle(0.1), shooter),
                        Commands.run(shooter::enable, shooter),
                        Commands.waitUntil(shooter::isSpinningAtVelocity),
                        Commands.run(indexer::transferToShooter, indexer),
                        Commands.waitSeconds(0.5),
                        Commands.run(shooter::disable, shooter)
                ).onlyIf(this::hasIndexerCollected)
        );
    }

    private boolean hasIndexerCollected() {
        boolean collected = indexer.hasCollected();
        if (!collected)
            logger.warn("Shoot called without a note collected! Interrupting...");
        return collected;
    }
}
