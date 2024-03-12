package org.tahomarobotics.robot.shooter.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

public class RedunShootCommand extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(ShootCommand.class);

    private final Indexer indexer = Indexer.getInstance();

    public RedunShootCommand() {
        Shooter shooter = Shooter.getInstance();

        addCommands(
                Commands.sequence(
                        Commands.runOnce(indexer::transitionToTransferring),
                        Commands.waitSeconds(0.1),
                        Commands.runOnce(() -> shooter.setAngle(ShooterConstants.SHOOTER_COLLECT_PIVOT_ANGLE)),
                        Commands.runOnce(shooter::disableRedunShootMode)
                ).onlyIf(shooter::inRedundantShootingMode)
        );
    }
}
