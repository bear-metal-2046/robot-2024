package org.tahomarobotics.robot.climbers.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.chassis.commands.HitSomething;
import org.tahomarobotics.robot.climbers.Climbers;

/**
 * Pathfinds the pre-climbed robot to the chain, puts the Amp Arm up, the drives forward to the final distance
 */
public class EngageCommand extends SequentialCommandGroup {
    public EngageCommand() {
        Climbers climbers = Climbers.getInstance();
        AmpArm ampArm = AmpArm.getInstance();

        addCommands(
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.ENGAGED)),
                AutoBuilder.pathfindToPose(ChassisConstants.getClosestChainPose(), ChassisConstants.CLIMB_MOVEMENT_CONSTRAINTS),
                AmpArmCommands.ARM_TO_CLIMB.get(),
                Commands.runOnce(ampArm::shiftNote),
                Commands.waitUntil(ampArm::isRollerAtPosition),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.COLLECTED)),
                new HitSomething(-0.5)
        );
    }
}
