package org.tahomarobotics.robot.climbers.commands;


import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.chassis.commands.DriveForwardCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

import java.util.Set;

import static org.tahomarobotics.robot.climbers.ClimberConstants.*;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        super(
                Commands.parallel(
                        Commands.runOnce(() -> Shooter.getInstance().setAngle(ShooterConstants.MAX_PIVOT_ANGLE)),
                        Commands.defer(() -> AutoBuilder.pathfindToPose(ChassisConstants.getClosestChainPose(), ChassisConstants.CLIMB_MOVEMENT_CONSTRAINTS), Set.of(Chassis.getInstance()))
                ),
                new ClimbCommand(TOP_POSITION, UNLADEN_SLOT),
                new DriveForwardCommand(0.1666666666666, 1),
                AmpArmCommands.AMP_ARM_CLIMB.get(),
                new ClimbCommand(BOTTOM_POSITION, LADEN_SLOT)
        );
    }
}