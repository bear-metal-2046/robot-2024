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
                        Commands.waitUntil(shooter::isReadyToShoot),
                        Commands.runOnce(indexer::transitionToTransferring),
                        Commands.waitUntil(() -> !indexer.isTransferring()),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(shooter::disable)
                ).onlyIf(shooter::inShootingMode)
        );
    }
}
