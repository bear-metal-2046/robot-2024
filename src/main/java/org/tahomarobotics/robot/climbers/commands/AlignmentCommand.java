package org.tahomarobotics.robot.climbers.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.chassis.ChassisConstants;

/**
 * Pathfinds the pre-climbed robot to the chain, puts the Amp Arm up, the drives forward to the final distance
 */
public class AlignmentCommand extends SequentialCommandGroup {
    public AlignmentCommand() {
        super(
                AutoBuilder.pathfindToPose(ChassisConstants.getClosestChainPose(), ChassisConstants.CLIMB_MOVEMENT_CONSTRAINTS)
        );
    }
}
