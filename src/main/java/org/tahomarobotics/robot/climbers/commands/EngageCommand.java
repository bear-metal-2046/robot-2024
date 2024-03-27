package org.tahomarobotics.robot.climbers.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OI;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.chassis.commands.DriveForwardCommand;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;
import org.tahomarobotics.robot.util.SafeAKitLogger;

import static org.tahomarobotics.robot.climbers.ClimberConstants.PARTIAL_CLIMB_POSITION;

/**
 * Pathfinds the pre-climbed robot to the chain, puts the Amp Arm up, the drives forward to the final distance
 */
public class EngageCommand extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(EngageCommand.class);

    public EngageCommand() {
        Climbers climbers = Climbers.getInstance();
        AmpArm ampArm = AmpArm.getInstance();
        Chassis chassis = Chassis.getInstance();

        Pose2d target = ChassisConstants.getClosestChainPose();
        double distance = target.getTranslation().getDistance(chassis.getPose().getTranslation());

        SafeAKitLogger.recordOutput("Climbers/Distance", distance);

        if (distance < ClimberConstants.CLOSENESS_THRESHOLD) {
            OI.getInstance().rumbleDrive();
            return;
        }

        addCommands(
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.ENGAGING)),
                Commands.runOnce(() -> logger.info("Pathfinding To Pre-climb Pose")),
                AutoBuilder.pathfindToPose(target, ChassisConstants.CLIMB_MOVEMENT_CONSTRAINTS),
                AmpArmCommands.ARM_TO_CLIMB.get().withTimeout(0.5)
        );

        if (climbers.isTrapping())
            addCommands(
                    Commands.runOnce(ampArm::shiftNote),
                    Commands.runOnce(() -> logger.info("Shifted Note Back")),
                    Commands.waitUntil(ampArm::isRollerAtPosition),
                    Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.COLLECTED))
            );

        addCommands(
                new DriveForwardCommand(-0.5,1.5),
                new LadenClimbCommand(PARTIAL_CLIMB_POSITION).alongWith(new DriveForwardCommand(0.5,0.25)),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.ENGAGED))
        );
    }
}
