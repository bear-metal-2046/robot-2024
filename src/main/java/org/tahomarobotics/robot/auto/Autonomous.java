package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.commands.ShootCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

public class Autonomous extends SubsystemIF {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(Autonomous.class);
    private static final Autonomous INSTANCE = new Autonomous();

    private final Chassis chassis = Chassis.getInstance();
    private final LoggedDashboardChooser<Command> autoChooser;

    private Autonomous() {
        Shooter shooter = Shooter.getInstance();
        Collector collector = Collector.getInstance();
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();

        NamedCommands.registerCommand("Shoot",
                Commands.race(Commands.waitUntil(indexer::hasCollected), Commands.waitSeconds(1)).andThen(Commands.runOnce(shooter::enableShootMode))
                        .andThen(new ShootCommand()));

        NamedCommands.registerCommand("SpinUp", Commands.runOnce(shooter::enable));

        NamedCommands.registerCommand("CollectorDown", Commands.runOnce(collector::setDeployed)
                .andThen(Commands.runOnce(() -> collector.setCollectionState(Collector.CollectionState.COLLECTING))));

        NamedCommands.registerCommand("CollectorUp",
                Commands.runOnce(collector::toggleDeploy));

        NamedCommands.registerCommand("AmpArmToScore",
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.AMP)));

        NamedCommands.registerCommand("AmpArmEject",
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.SCORE))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)));

        NamedCommands.registerCommand("AmpArmToStow",
                Commands.runOnce(() -> ampArm.setArmState(AmpArm.ArmState.STOW))
                        .andThen(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)));

        autoChooser = PathPlannerHelper.getAutoChooser(chassis, this::onAutoChange);

        // TODO: Test this with DriverStation;
        //  as the simulation interface does allow for changing it when not disconnected.
        var inst = NetworkTableInstance.getDefault();
        inst.addListener(
                inst.getBooleanTopic("/FMSInfo/IsRedAlliance").getEntry(true),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> this.onAutoChange(autoChooser.get().getName())
        );
    }

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    public Command getSelectedAuto() {
        return autoChooser.get();
    }

    // PATH VISUALIZATION

    private static List<PathPlannerTrajectory> getTrajectories(List<PathPlannerPath> autoPaths) {
        List<PathPlannerTrajectory> autoTrajectories = new ArrayList<>();

        ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
        Rotation2d lastRotation = new Rotation2d();

        for (PathPlannerPath path : autoPaths) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                path = path.flipPath();
            }

            PathPlannerTrajectory pathTrajectory = path.getTrajectory(lastChassisSpeeds, lastRotation);
            autoTrajectories.add(pathTrajectory);

            PathPlannerTrajectory.State endState = pathTrajectory.getEndState();

            lastChassisSpeeds = new ChassisSpeeds(
                    endState.heading.getSin() * endState.velocityMps,
                    endState.heading.getCos() * endState.velocityMps,
                    (endState.holonomicAngularVelocityRps.isPresent()) ? endState.holonomicAngularVelocityRps.get() : 0
            );
            lastRotation = endState.targetHolonomicRotation;
        }

        return autoTrajectories;
    }

    private static Trajectory convertToTrajectory(List<PathPlannerPath> selectedAutoPaths) {
        return new Trajectory(
                getTrajectories(selectedAutoPaths).stream().flatMap(
                        trajectory ->
                                trajectory
                                        .getStates().stream().map
                                                (state -> new Trajectory.State(
                                                        state.timeSeconds,
                                                        state.velocityMps,
                                                        state.accelerationMpsSq,
                                                        state.getTargetHolonomicPose(), // This will be slightly inaccurate.
                                                        state.curvatureRadPerMeter))
                ).toList());
    }

    public static Command skipToNextPath(PathPlannerAuto auto) {
        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(auto.getName());
        List<PathPlannerTrajectory> trajectories = getTrajectories(paths);
        Pose2d currentPose = Chassis.getInstance().getPose();
        Pair<Translation2d, Double> closestPose = Pair.of(currentPose.getTranslation(), Double.POSITIVE_INFINITY);
        PathPlannerTrajectory closestTraj = paths.get(0).getTrajectory(new ChassisSpeeds(), new Rotation2d());
        double distance;
        for (PathPlannerTrajectory traj : trajectories) {
            for (Translation2d pos : (Translation2d[]) traj.getStates().stream().map((state) -> state.positionMeters).toArray()) {
                distance = pos.getDistance(currentPose.getTranslation());
                if (distance <= closestPose.getSecond()) {
                    closestPose = Pair.of(pos, distance);
                    closestTraj = traj;
                }
            }
        }
        PathPlannerPath targetPath = paths.get(0);
        for (PathPlannerPath path : paths) {
            if (path.getPoint(0).position.equals(closestTraj.getState(0).positionMeters)) {
                targetPath = path;
            }
        }
        return AutoBuilder.pathfindThenFollowPath(targetPath, ChassisConstants.AUTO_PATHFINDING_CONSTRAINTS);
    }

    private void postAutoTrajectory(Field2d field, String autoName) {
        if (autoName == null || autoName.equals(AutoConstants.DEFAULT_AUTO_NAME)) {
            chassis.resetOdometry(new Pose2d());
            field.getObject("Trajectory").setTrajectory(new Trajectory());
            return;
        }

        var autoPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        if (!autoPaths.isEmpty()) {
            var firstPath = autoPaths.get(0);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                firstPath = firstPath.flipPath();
            }
            var startingPose = firstPath.getPreviewStartingHolonomicPose();
            chassis.resetOdometry(startingPose);
        }
        field.getObject("Trajectory").setTrajectory(convertToTrajectory(autoPaths));
    }

    private void onAutoChange(String autoName) {
        postAutoTrajectory(chassis.getField(), autoName);
    }
}
