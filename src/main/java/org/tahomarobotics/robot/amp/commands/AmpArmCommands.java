package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

import java.util.Set;
import java.util.function.Supplier;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;

public class AmpArmCommands {
    public static final Supplier<Command> FEEDFORWARD;
    public static final Supplier<Command> FEEDBACK;
    private static final Supplier<Command> STOW_TO_AMP;
    private static final Supplier<Command> STOW_TO_SOURCE;
    public static final Supplier<Command> ARM_TO_STOW;
    public static final Supplier<Command> AMP_ARM_CLIMB;
    public static final Supplier<Command> AMP_ARM_TRAP;
    public static Command AMP_ARM_CTRL;

    static {
        AmpArm ampArm = AmpArm.getInstance();

        STOW_TO_AMP = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_AMP_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP))
        ).onlyIf(ampArm::isArmAtStow);

        ARM_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_STOW_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition).alongWith(
                        Commands.waitUntil(() -> ampArm.getArmPosition() < 0.0)
                                .andThen(Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW))))
        );

        STOW_TO_SOURCE = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_SOURCE_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.SOURCE))
        ).onlyIf(ampArm::isArmAtStow);

        AMP_ARM_CLIMB = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.CLIMB)),
                Commands.waitUntil(ampArm::isArmAtPosition)
        );

        AMP_ARM_TRAP = () -> Commands.sequence(
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.TRAP)),
//                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.waitUntil(ampArm::isWristAtPosition)
        );
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        FEEDFORWARD = () -> Commands.sequence(
                Commands.runOnce(shooter::stop),
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
                    shooter.stop();
                })
        ).onlyIf(ampArm::isArmAtStow).onlyIf(indexer::hasCollected);

        FEEDBACK = () -> Commands.sequence(
                Commands.runOnce(() -> shooter.setAngle(ShooterConstants.MIN_PIVOT_ANGLE)),
                Commands.runOnce(() -> {
                    shooter.reverseIntake();
                    indexer.transitionToReverseIntaking();
                }),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.REVERSE_INTAKE)),
                Commands.waitUntil(indexer::getCollectorBeanBake).withTimeout(2.0),
                Commands.waitSeconds(0.05),
                Commands.runOnce(() -> {
                    ampArm.setRollerState(AmpArm.RollerState.DISABLED);
                    indexer.transitionToCollected();
                    shooter.stop();
                    ampArm.setArmState(AmpArm.ArmState.STOW);
                })
        ).onlyIf(() -> !indexer.hasCollected()).onlyIf(ampArm::isRollerCollected).onlyIf(ampArm::isArmAtStow);
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        AMP_ARM_CTRL = Commands.deferredProxy(() -> {
            if ((ampArm.isArmAtSource() || ampArm.isArmAtAmp()) && ampArm.isRollerCollected()) {
                // TODO: But pass back back in once the shooter is fixed.
                return Commands.defer(ARM_TO_STOW, Set.of(ampArm, indexer, shooter));
            } if (ampArm.isArmAtAmp() || ampArm.isArmAtSource() || ampArm.isArmAtTrap()) {
                return Commands.defer(ARM_TO_STOW, Set.of(ampArm, shooter));
            } if (indexer.hasCollected()) {
                return Commands.defer(() -> FEEDFORWARD.get().andThen(STOW_TO_AMP.get()), Set.of(ampArm, indexer, shooter));
            } else if (ampArm.isRollerCollected()) {
                return Commands.defer(STOW_TO_AMP, Set.of(ampArm));
            } else {
                return Commands.defer(() -> STOW_TO_SOURCE.get().andThen(Commands.runOnce(shooter::stop)), Set.of(ampArm));
            }
        });
    }
}
