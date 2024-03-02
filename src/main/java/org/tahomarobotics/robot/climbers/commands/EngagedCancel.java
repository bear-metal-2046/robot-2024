package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.climbers.Climbers;

public class EngagedCancel extends SequentialCommandGroup {
    public EngagedCancel() {
        Climbers climbers = Climbers.getInstance();

        addCommands(
                Commands.runOnce(() -> CommandScheduler.getInstance().requiring(Chassis.getInstance()).cancel()),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.READY))
        );
    }
}
