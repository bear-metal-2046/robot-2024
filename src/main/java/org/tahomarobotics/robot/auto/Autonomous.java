package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.commands.ShootCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

public class Autonomous extends SubsystemIF {
    private static final Autonomous INSTANCE = new Autonomous();
    private final Chassis chassis = Chassis.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final LoggedDashboardChooser<Command> autoChooser;
    private final Field2d fieldPose;

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    private Autonomous() {
        autoChooser = PathPlannerHelper.getAutoChooser(chassis);

        NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
        StringSubscriber autoSub = netInstance.getTable("SmartDashboard/Auto").getStringTopic("selected").subscribe("Test Auto");
        BooleanSubscriber allianceChange = netInstance.getTable("FMSInfo").getBooleanTopic("IsRedAlliance").subscribe(true);
        fieldPose = chassis.getField();
        netInstance.addListener(
                autoSub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> new InstantCommand(() -> postAutoTrajectory(fieldPose, autoSub.get())).ignoringDisable(true).schedule()
        );
        netInstance.addListener(
                allianceChange,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> new InstantCommand(() -> postAutoTrajectory(fieldPose, autoChooser.get().getName())).ignoringDisable(true).schedule()
        );

        NamedCommands.registerCommand("Shoot",
                Commands.runOnce(shooter::toggleShootMode).onlyIf(() -> !shooter.inShootingMode())
                        .andThen(new ShootCommand())
                        .andThen(shooter::toggleShootMode));
        NamedCommands.registerCommand("ResetOdom", Commands.runOnce(() -> chassis.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.get().getName()))));
        NamedCommands.registerCommand("CollectorDown",
                Commands.runOnce(collector::toggleDeploy).onlyIf(() -> !collector.isDeployed())
                        .andThen(() -> collector.setCollectionState(Collector.CollectionState.COLLECTING)));
        NamedCommands.registerCommand("CollectorUp",
                Commands.runOnce(collector::stopCollect)
                        .andThen(Commands.runOnce(collector::toggleDeploy).onlyIf(collector::isDeployed)));
    }

    public Command getSelectedAuto() {
        return autoChooser.get();
    }

    public Trajectory convertToTrajectory(List<PathPlannerPath> selectedAutoPaths) {
        List<Trajectory> autoTrajectories = getTrajectories(selectedAutoPaths).stream().map  // PathPlannerTrajectories are
                ((trajectory) -> new Trajectory(                                             // essentially still Trajectories,
                        trajectory                                                           // this just puts the values
                                .getStates().stream().map                                    // in the right places
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
            finishedTrajectory = finishedTrajectory.concatenate(trajectory);
        }
        return finishedTrajectory;
    }

    private List<PathPlannerTrajectory> getTrajectories(List<PathPlannerPath> autoPaths) {
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

    public void postAutoTrajectory(Field2d field, String autoName) {
        field.getObject("Trajectory").setTrajectory(convertToTrajectory(PathPlannerAuto.getPathGroupFromAutoFile(autoName)));
    }

    @Override
    public void onDisabledInit() {
        postAutoTrajectory(chassis.getField(), getSelectedAuto().getName());
    }
}
