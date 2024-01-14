package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;

public class PathPlannerHelper {
    public static SendableChooser<Command> getAutoChooser() {
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
                    if (alliance.isPresent()) {
                        return alliance.get().equals(DriverStation.Alliance.Red);
                    }
                    return false;
                },
                chassis
        );

        NamedCommands.registerCommand("ShootCommand", new InstantCommand(() -> System.out.println("========SHOOT COMMAND CALLED=========")));
        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        return autoChooser;
    }
}
