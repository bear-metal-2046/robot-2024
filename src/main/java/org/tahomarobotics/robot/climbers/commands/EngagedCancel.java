package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.climbers.Climbers;

public class EngagedCancel extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(EngagedCancel.class);

    public EngagedCancel() {
        Climbers climbers = Climbers.getInstance();

        addCommands(
                Commands.runOnce(() -> logger.info("Engaged Command Canceled")),
                Commands.runOnce(() -> CommandScheduler.getInstance().requiring(Chassis.getInstance()).cancel()),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.READY))
        );
    }
}
