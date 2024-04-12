package org.tahomarobotics.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

public class ShootCommand extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(ShootCommand.class);

    public ShootCommand() {
        Shooter shooter = Shooter.getInstance();
        Indexer indexer = Indexer.getInstance();

        addCommands(
                Commands.sequence(
                        Commands.waitUntil(shooter::isReadyToShoot),
                        Commands.runOnce(() -> logger.info("Shooting")),
                        Commands.runOnce(indexer::transitionToTransferring),
                        Commands.waitSeconds(0.2),
                        Commands.runOnce(shooter::disableReadyMode),
                        Commands.runOnce(() -> shooter.setAngle(ShooterConstants.SHOOTER_COLLECT_PIVOT_ANGLE)),
                        Commands.runOnce(() -> logger.info("Shot"))
                ).onlyIf(shooter::inReadyMode)
        );
    }
}
