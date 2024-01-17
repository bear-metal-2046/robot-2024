package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;

public class PathPlannerHelper {
    public static LoggedDashboardChooser<Command> getAutoChooser() {
        Chassis chassis = Chassis.getInstance();
        AutoBuilder.configureHolonomic(
                chassis::getPose,
                chassis::resetOdometry,
                chassis::getCurrentChassisSpeeds,
                chassis::autoDrive,
                new HolonomicPathFollowerConfig(
                        ChassisConstants.AUTO_TRANSLATION_PID,
                        ChassisConstants.AUTO_ROTATION_PID,
                        ChassisConstants.MAX_VELOCITY,
                        ChassisConstants.TRACK_WIDTH/2,
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.map(value -> value.equals(DriverStation.Alliance.Red)).orElse(false);
                },
                chassis
        );

        NamedCommands.registerCommand("ShootCommand", new InstantCommand(() -> System.out.println("========SHOOT COMMAND CALLED=========")));

        return new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
    }
}
