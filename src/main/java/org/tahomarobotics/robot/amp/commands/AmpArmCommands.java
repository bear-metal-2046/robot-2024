package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;

import java.util.Set;
import java.util.function.Supplier;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;

public class AmpArmCommands {
    private static final Supplier<Command> FEEDFORWARD;
    private static final Supplier<Command> FEEDBACK;
    private static final Supplier<Command> STOW_TO_AMP;
    private static final Supplier<Command> STOW_TO_SOURCE;
    private static final Supplier<Command> ARM_TO_STOW;
    public static Command AMP_ARM_CTRL;

    static {
        AmpArm ampArm = AmpArm.getInstance();

        STOW_TO_AMP = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_AMP_POSE))
        ).onlyIf(ampArm::isStowed);

        ARM_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_STOW_POSE))
        ).onlyIf(() -> ampArm.isAmp() || ampArm.isSource());

        STOW_TO_SOURCE = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.SOURCE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_SOURCE_POSE))
        ).onlyIf(ampArm::isStowed);
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        FEEDFORWARD = () -> Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.transferToAmp();
                    ampArm.setRollerState(AmpArm.RollerState.PASSING);
                }),
                Commands.waitSeconds(0.1),
                Commands.runOnce(indexer::transitionToTransferring),
                Commands.waitUntil(() -> !indexer.isBeanBakeTwo() && !indexer.isBeanBakeOne()),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.COLLECTED);
                    indexer.transitionToDisabled();
                    shooter.disable();
                }),
                STOW_TO_AMP.get()
        ).onlyIf(ampArm::isStowed).onlyIf(indexer::hasCollected);

        FEEDBACK = () -> Commands.sequence(
                ARM_TO_STOW.get(),
                Commands.waitUntil(ampArm::isWristAtPosition),
                Commands.runOnce(() -> {
                    shooter.reverseIntake();
                    indexer.transitionToReverseIntaking();
                }),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.SCORE)),
                Commands.waitUntil(indexer::isBeanBakeOne).withTimeout(2.0),
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
            if ((ampArm.isSource() || ampArm.isAmp()) && ampArm.isCollected())
                return Commands.defer(FEEDBACK, Set.of(ampArm, indexer, shooter));
            if (ampArm.isAmp() || ampArm.isSource())
                return Commands.defer(ARM_TO_STOW, Set.of(ampArm));
            if (indexer.hasCollected())
                return Commands.defer(FEEDFORWARD, Set.of(ampArm, indexer, shooter));
            else
                return Commands.defer(STOW_TO_SOURCE, Set.of(ampArm));
        });
    }
}
