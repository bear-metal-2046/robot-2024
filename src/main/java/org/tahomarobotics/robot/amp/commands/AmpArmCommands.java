package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;

import java.util.Set;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;

public class AmpArmCommands {
    public static Command PASS_THROUGH;
    public static Command STOW_TO_AMP;
    public static Command STOW_TO_SOURCE;
    public static Command COLLECT_FROM_SOURCE;

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        PASS_THROUGH = Commands.defer(() -> Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.transferToAmp();
                    ampArm.setRollerState(AmpArm.RollerState.PASSING);
                }),
                Commands.waitSeconds(0.25),
                Commands.runOnce(indexer::transitionToTransferring),
                Commands.waitSeconds(0.6),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.COLLECTED);
                    indexer.transitionToDisabled();
                    shooter.disable();
                })
        ).onlyIf(ampArm::isStowed).onlyIf(indexer::hasCollected), Set.of(ampArm, indexer, shooter));

        COLLECT_FROM_SOURCE = Commands.defer(() -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_STOW_POSE)),
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW)),
                Commands.waitUntil(ampArm::isWristAtPosition),
                Commands.runOnce(() -> {
                    shooter.reverseIntake();
                    indexer.transitionToReverseIntaking();
                }),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.SCORE)),
                Commands.waitUntil(indexer::isBeamBroken),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.DISABLED);
                    indexer.transitionToDisabled();
                    shooter.disable();
                })
        ).onlyIf(ampArm::isSource).onlyIf(() -> !indexer.hasCollected()), Set.of(ampArm, indexer, shooter));
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();

        STOW_TO_AMP = Commands.defer(() -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_AMP_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP))
        ).onlyIf(ampArm::isStowed), Set.of(ampArm));

        STOW_TO_SOURCE = Commands.defer(() -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_SOURCE_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.SOURCE))
        ).onlyIf(ampArm::isStowed), Set.of(ampArm));
    }
}
