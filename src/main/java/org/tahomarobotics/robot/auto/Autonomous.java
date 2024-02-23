package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.commands.ZeroCollectorCommand;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.commands.ShootCommand;
import org.tahomarobotics.robot.shooter.commands.ZeroShooterCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.io.File;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

public class Autonomous extends SubsystemIF {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(Autonomous.class);
    private static final Autonomous INSTANCE = new Autonomous();
    private final Chassis chassis = Chassis.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final AmpArm ampArm = AmpArm.getInstance();
    private final LoggedDashboardChooser<Command> autoChooser;
    private final Field2d fieldPose;

    private Optional<DriverStation.Alliance> prevAlliance = DriverStation.getAlliance();

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    private Autonomous() {

        NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
        StringSubscriber autoSub = netInstance.getTable("SmartDashboard/Auto").getStringTopic("selected").subscribe("");
        fieldPose = chassis.getField();
        netInstance.addListener(
                autoSub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> new InstantCommand(() -> postAutoTrajectory(fieldPose, autoSub.get())).ignoringDisable(true).schedule()
        );

        NamedCommands.registerCommand("Shoot",
                Commands.race(Commands.waitUntil(indexer::hasCollected), Commands.waitSeconds(1)).andThen(Commands.runOnce(shooter::enable))
                        .andThen(new ShootCommand()));

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

        autoChooser = PathPlannerHelper.getAutoChooser(chassis);
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
        if (autoName.equalsIgnoreCase("InstantCommand") || autoName.isEmpty()) {
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

    @Override
    public void periodic() {
        var alliance = DriverStation.getAlliance();
        if (prevAlliance.isPresent() != alliance.isPresent() || alliance.isPresent() && (prevAlliance.get() != alliance.get())) {
            prevAlliance = alliance;
            postAutoTrajectory(chassis.getField(), getSelectedAuto().getName());
        }
    }
}
