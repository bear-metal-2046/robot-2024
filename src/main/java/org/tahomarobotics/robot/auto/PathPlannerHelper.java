package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;

import java.util.function.Consumer;

public class PathPlannerHelper {
    public static LoggedDashboardChooser<Command> getAutoChooser(Chassis chassis, Consumer<String> onChange) {
        AutoBuilder.configureHolonomic(
                chassis::getPose,
                chassis::resetOdometry,
                chassis::getCurrentChassisSpeeds,
                chassis::autoDrive,
                new HolonomicPathFollowerConfig(
                        ChassisConstants.AUTO_TRANSLATION_PID,
                        ChassisConstants.AUTO_ROTATION_PID,
                        ChassisConstants.MAX_VELOCITY,
                        ChassisConstants.DRIVE_RADIUS,
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.map(value -> value.equals(DriverStation.Alliance.Red)).orElse(false);
                },
                chassis
        );

        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Auto");

        Command defaultCommand = Commands.none().withName(AutoConstants.DEFAULT_AUTO_NAME);

        chooser.addDefaultOption(AutoConstants.DEFAULT_AUTO_NAME, defaultCommand);
        AutoBuilder.getAllAutoNames().stream().map(PathPlannerAuto::new)
                .forEach(auto -> chooser.addOption(auto.getName(), auto));

        chooser.getSendableChooser().onChange(onChange);

        return chooser;
    }

}
