package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;

import java.util.function.Supplier;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;

public class AmpArmCommands {
    public static Supplier<Command> FEEDFORWARD;
    public static Supplier<Command> STOW_TO_AMP;
    public static Supplier<Command> AMP_TO_STOW;
    public static Supplier<Command> STOW_TO_SOURCE;
    public static Supplier<Command> FEEDBACK;

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
                Commands.waitUntil(() -> !indexer.isBeanBakeTwo()),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.COLLECTED);
                    indexer.transitionToDisabled();
                    shooter.disable();
                })
        ).onlyIf(ampArm::isStowed).onlyIf(indexer::hasCollected);

        FEEDBACK = () -> Commands.sequence(
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
                Commands.waitUntil(indexer::isBeanBakeOne),
                Commands.waitSeconds(0.1),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.DISABLED);
                    indexer.transitionToCollected();
                    shooter.disable();
                })
        ).onlyIf(() -> !indexer.hasCollected());
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();

        STOW_TO_AMP = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_AMP_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP))
        ).onlyIf(ampArm::isStowed);

        AMP_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_STOW_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW))
        ).onlyIf(ampArm::isAmp);

        STOW_TO_SOURCE = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_SOURCE_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.SOURCE))
        ).onlyIf(ampArm::isStowed);
    }
}
