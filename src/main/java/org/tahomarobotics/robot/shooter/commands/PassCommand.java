package org.tahomarobotics.robot.shooter.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.auto.Autonomous;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.shooter.Shooter;

public class PassCommand extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(PassCommand.class);

    public PassCommand() {
        Shooter shooter = Shooter.getInstance();
        Indexer indexer = Indexer.getInstance();
        AmpArm ampArm = AmpArm.getInstance();

        addCommands(
                Commands.sequence(
                        Commands.waitUntil(shooter::isReadyToShoot),
//                        Commands.sequence(
//                            Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.PASSING)),
//                            Commands.waitUntil(ampArm::isArmAtAmp)
//                          ).onlyIf(() -> shooter.getShootMode().equals(Shooter.ShootMode.PASSING_LOW)),
                        Commands.runOnce(() -> logger.info("Passing")),
                        Commands.runOnce(indexer::transitionToTransferring),
                        Commands.waitSeconds(0.2),
                        Commands.runOnce(shooter::disableShootMode),
//                        Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW))
//                          .onlyIf(() -> shooter.getShootMode().equals(Shooter.ShootMode.PASSING_LOW)),
                        Commands.runOnce(() -> shooter.setAngle(ShooterConstants.SHOOTER_COLLECT_PIVOT_ANGLE)),
                        Commands.runOnce(() -> logger.info("Passed"))
                ).onlyIf(shooter::inPassingMode)
        );
    }
}
