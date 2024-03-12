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

    public static final Supplier<Command> FEEDFORWARD;
    public static final Supplier<Command> FEEDBACK;
    private static final Supplier<Command> ARM_TO_AMP;
    private static final Supplier<Command> ARM_TO_SOURCE;
    public static final Supplier<Command> ARM_TO_STOW;
    public static final Supplier<Command> ARM_TO_CLIMB;
    public static final Supplier<Command> ARM_TO_TRAP;
    public static Command AMP_ARM_CTRL;

    static {
        AmpArm ampArm = AmpArm.getInstance();

        ARM_TO_AMP = () -> Commands.sequence(
                Commands.runOnce(() -> logger.info("Arm To Amp")),
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_AMP_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition).alongWith(
                        Commands.waitUntil(() -> ampArm.getArmPosition() > WRIST_MOVING_POSE_THRESHOLD)
                                .andThen(Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP))))
        );

        ARM_TO_STOW = () -> Commands.sequence(
                Commands.runOnce(() -> logger.info("Arm To Stow")),
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_STOW_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition).alongWith(
                        Commands.waitUntil(() -> ampArm.getArmPosition() < -WRIST_MOVING_POSE_THRESHOLD)
                                .andThen(Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW))))
        );

        ARM_TO_SOURCE = () -> Commands.sequence(
                Commands.runOnce(() -> logger.info("Arm To Source")),
                Commands.runOnce(() -> ampArm.setWristPosition(WRIST_MOVING_POSE)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_SOURCE_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition).alongWith(
                        Commands.waitUntil(() -> ampArm.getArmPosition() > WRIST_MOVING_POSE_THRESHOLD)
                                .andThen(Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.SOURCE))))
        );

        ARM_TO_CLIMB = () -> Commands.sequence(
                Commands.runOnce(() -> logger.info("Arm To Climb")),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.CLIMB)),
                Commands.waitUntil(ampArm::isArmAtPosition)
        );

        ARM_TO_TRAP = () -> Commands.sequence(
                Commands.runOnce(() -> logger.info("Arm To Trap")),
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.TRAP)),
                Commands.waitUntil(ampArm::isWristAtPosition)
        );
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        FEEDFORWARD = () -> Commands.sequence(
                Commands.runOnce(() -> logger.info("Feeding Forward")),
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
        ).onlyIf(ampArm::isArmAtStow).onlyIf(indexer::isCollected);

        FEEDBACK = () -> Commands.sequence(
                Commands.runOnce(() -> logger.info("Feeding Back")),
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
        ).onlyIf(() -> !indexer.isCollected()).onlyIf(ampArm::isRollerCollected).onlyIf(ampArm::isArmAtStow);
    }

    static {
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();

        AMP_ARM_CTRL = Commands.deferredProxy(() -> {
            if ((ampArm.isArmAtSource() || ampArm.isArmAtAmp()) && ampArm.isRollerCollected()) {
                return Commands.defer(ARM_TO_STOW, Set.of(ampArm, indexer, shooter));
            } if (ampArm.isArmAtAmp() || ampArm.isArmAtSource() || ampArm.isArmAtTrap()) {
                return Commands.defer(ARM_TO_STOW, Set.of(ampArm, shooter));
            } if (indexer.isCollected()) {
                return Commands.defer(() -> FEEDFORWARD.get().andThen(ARM_TO_AMP.get()), Set.of(ampArm, indexer, shooter));
            } else if (ampArm.isRollerCollected()) {
                return Commands.defer(ARM_TO_AMP, Set.of(ampArm));
            } else {
                return Commands.defer(() -> ARM_TO_SOURCE.get().andThen(Commands.runOnce(shooter::stop)), Set.of(ampArm));
            }
        });
    }
}
