package org.tahomarobotics.robot.amp.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.OI;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.AmpArmConstants;

import java.util.function.BooleanSupplier;

public class SourceIntakeCommand extends SequentialCommandGroup {
    public SourceIntakeCommand(BooleanSupplier trigger) {
        AmpArm ampArm = AmpArm.getInstance();
        Timer timer = new Timer();

        addCommands(
                Commands.runOnce(timer::reset),
                Commands.runOnce(timer::start),
            Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.PASSING)),
            Commands.waitUntil(() -> (ampArm.getRollerCurrent() > AmpArmConstants.SOURCE_COLLECT_CURRENT && timer.hasElapsed(AmpArmConstants.SOURCE_COLLECT_ACCEL_TIME*2)) || !trigger.getAsBoolean()), Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)),
                Commands.sequence(
                        Commands.runOnce(() -> OI.getInstance().log("DETECTED NOTE")),
                Commands.runOnce(ampArm::sourceIntake),
                Commands.waitUntil(ampArm::isRollerAtPosition),
                    Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.COLLECTED))
            ).onlyIf(trigger)
        );
    }
}
