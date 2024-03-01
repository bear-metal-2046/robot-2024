package org.tahomarobotics.robot.climbers.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.chassis.commands.DriveForwardCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

import static org.tahomarobotics.robot.climbers.ClimberConstants.*;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        super(
                Commands.runOnce(() -> Shooter.getInstance().setAngle(ShooterConstants.MAX_PIVOT_ANGLE)),
//                Commands.parallel(
////                        Commands.defer(() -> AutoBuilder.pathfindToPose(ChassisConstants.getClosestChainPose(), ChassisConstants.CLIMB_MOVEMENT_CONSTRAINTS), Set.of(Chassis.getInstance()))
//                ),
                new ClimbCommand(TOP_POSITION, UNLADEN_SLOT),
                new DriveForwardCommand(-0.35, 1),
                Commands.parallel(
                        AmpArmCommands.AMP_ARM_CLIMB.get(),
                        new DriveForwardCommand(-0.35, 1)
                ),
                new ClimbCommand(BOTTOM_POSITION, LADEN_SLOT)
        );
    }
}