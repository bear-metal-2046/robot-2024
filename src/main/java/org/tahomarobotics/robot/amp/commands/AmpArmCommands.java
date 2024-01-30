package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;

import java.util.Set;

import static org.tahomarobotics.robot.amp.AmpArmConstants.ARM_AMP_POSE;
import static org.tahomarobotics.robot.amp.AmpArmConstants.WRIST_MOVING_POSE;

public class AmpArmCommands {
    public static Command PASS_THROUGH;
    public static Command STOW_TO_AMP;

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        PASS_THROUGH = Commands.defer(() -> Commands.sequence(
                Commands.runOnce(() -> {
                    shooter.transferToAmp();
                    ampArm.setRollersState(AmpArm.RollerState.COLLECT);
                }),
                Commands.waitSeconds(0.25),
                Commands.runOnce(indexer::transitionToTransferring),
                Commands.waitSeconds(0.6),
                Commands.runOnce(() -> {
                    ampArm.setRollersState(AmpArm.RollerState.DISABLED);
                    indexer.transitionToDisabled();
                    shooter.disable();
                })
        ).onlyIf(ampArm::isStowed), Set.of(ampArm, indexer, shooter));
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();

        STOW_TO_AMP = Commands.defer(() -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_AMP_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP))
        ).onlyIf(ampArm::isStowed), Set.of(ampArm));
    }
}
