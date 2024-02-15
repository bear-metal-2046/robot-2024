package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

import java.util.Set;
import java.util.function.Supplier;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;

public class AmpArmCommands {
    private static final Logger logger = LoggerFactory.getLogger(AmpArmCommands.class);

    private static final Supplier<Command> FEEDFORWARD;
    private static final Supplier<Command> FEEDBACK;
    private static final Supplier<Command> STOW_TO_AMP;
    private static final Supplier<Command> STOW_TO_SOURCE;
    public static final Supplier<Command> ARM_TO_STOW;
    public static Command AMP_ARM_CTRL;

    static {
        AmpArm ampArm = AmpArm.getInstance();

        STOW_TO_AMP = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_AMP_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP))
        ).onlyIf(ampArm::isStowed);

        ARM_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_STOW_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW))
        );

        STOW_TO_SOURCE = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_SOURCE_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.SOURCE))
        ).onlyIf(ampArm::isStowed);
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        FEEDFORWARD = () -> Commands.sequence(
                Commands.runOnce(() -> shooter.setAngle(ShooterConstants.MIN_PIVOT_ANGLE)),
                Commands.waitUntil(shooter::isAtAngle),
                Commands.runOnce(() -> {
                    shooter.transferToAmp();
                    ampArm.setRollerState(AmpArm.RollerState.PASSING);
                }),
                Commands.waitSeconds(0.1),
                Commands.runOnce(indexer::transitionToTransferring),
                Commands.waitUntil(() -> !indexer.getShooterBeanBake() && !indexer.getCollectorBeanBake()),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.COLLECTED);
                    indexer.transitionToDisabled();
                    shooter.disable();
                })
        ).onlyIf(ampArm::isStowed).onlyIf(indexer::hasCollected);

        FEEDBACK = () -> Commands.sequence(
                Commands.runOnce(() -> shooter.setAngle(ShooterConstants.MIN_PIVOT_ANGLE)),
                Commands.waitUntil(shooter::isAtAngle),
                Commands.waitUntil(ampArm::isWristAtPosition),
                Commands.runOnce(() -> {
                    shooter.reverseIntake();
                    indexer.transitionToReverseIntaking();
                }),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.SCORE)),
                Commands.waitUntil(indexer::getCollectorBeanBake).withTimeout(2.0),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.DISABLED);
                    indexer.transitionToCollected();
                    shooter.disable();
                })
        ).onlyIf(() -> !indexer.hasCollected());
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        AMP_ARM_CTRL = Commands.deferredProxy(() -> {
            if ((ampArm.isSource() || ampArm.isAmp()) && ampArm.isCollected()) {
                logger.info("Running Arm to Stow then Feedback...");
                return Commands.defer(() -> ARM_TO_STOW.get().andThen(FEEDBACK.get()), Set.of(ampArm, indexer, shooter));
            } if (ampArm.isAmp() || ampArm.isSource()) {
                logger.info("Running Arm to Stow...");
                return Commands.defer(ARM_TO_STOW, Set.of(ampArm));
            } if (indexer.hasCollected()) {
                logger.info("Running Feedforward then Stow to Amp...");
                return Commands.defer(() -> FEEDFORWARD.get().andThen(STOW_TO_AMP.get()), Set.of(ampArm, indexer, shooter));
            } else {
                logger.info("Running Stow to Source...");
                return Commands.defer(STOW_TO_SOURCE, Set.of(ampArm));
            }
        });
    }
}