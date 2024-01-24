package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;

public class Autonomous extends SubsystemIF {
    private static final Autonomous INSTANCE = new Autonomous();
    private static final Chassis chassis = Chassis.getInstance();

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    LoggedDashboardChooser<Command> autoChooser = PathPlannerHelper.getAutoChooser();

    public Command getSelectedAuto() {
        return autoChooser.get();
    }

    public Trajectory convertToTrajectory(PathPlannerAuto selectedAuto) {
        List<Trajectory> autoTrajectories = getTrajectories(selectedAuto)
                .stream().map((trajectory) -> new Trajectory(
                        trajectory
                                .getStates().stream().map
                                        (state -> new Trajectory.State (
                                                state.timeSeconds,
                                                state.velocityMps,
                                                state.accelerationMpsSq,
                                                state.getTargetHolonomicPose(),
                                                state.curvatureRadPerMeter))
                                .toList()
                )).toList();
        Trajectory finishedTrajectory = new Trajectory();
        for (Trajectory trajectory : autoTrajectories) {
            finishedTrajectory.concatenate(trajectory);
        }
        return finishedTrajectory;
    }

    private List<PathPlannerTrajectory> getTrajectories(PathPlannerAuto auto) {
        List<PathPlannerPath> autoPaths = PathPlannerAuto.getPathGroupFromAutoFile(auto.getName());
        List<PathPlannerTrajectory> autoTrajectories = new ArrayList<>();
        ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
        Rotation2d lastRotation = new Rotation2d();
        for (PathPlannerPath path : autoPaths) {
            PathPlannerTrajectory pathTrajectory = path.getTrajectory(lastChassisSpeeds, lastRotation);
            autoTrajectories.add(pathTrajectory);
            PathPlannerTrajectory.State endState = pathTrajectory.getEndState();
            lastChassisSpeeds = new ChassisSpeeds(
                    endState.heading.getSin() * endState.velocityMps,
                    endState.heading.getCos() * endState.velocityMps,
                    0
            );
            lastRotation = endState.targetHolonomicRotation;
        }
        return autoTrajectories;
    }

    public void registerAutoCommands() {
        NamedCommands.registerCommand("ShootCommand", new InstantCommand(() -> System.out.println("========SHOOT COMMAND CALLED=========")));
        NamedCommands.registerCommand("ResetOdom", new InstantCommand(() -> chassis.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.get().getName()))));
    }
}
