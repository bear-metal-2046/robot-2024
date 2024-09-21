package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.commands.ShootCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

public class Autonomous extends SubsystemIF {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(Autonomous.class);
    private static final Autonomous INSTANCE = new Autonomous();

    private final Chassis chassis = Chassis.getInstance();
    private final LoggedDashboardChooser<Command> autoChooser;

    private int shotNumber = 0;
    private boolean useLookupTable = true;
    private String currentAutoName = AutoConstants.DEFAULT_AUTO_NAME;
    private double[] currentAutoAngles = new double[]{};

    private Autonomous() {
        Shooter shooter = Shooter.getInstance();
        Collector collector = Collector.getInstance();
        AmpArm ampArm = AmpArm.getInstance();
        Indexer indexer = Indexer.getInstance();

        NamedCommands.registerCommand("ZeroCollectorShooter", Commands.runOnce(() -> {
            shooter.zero();
            collector.zeroCollector();
        }).onlyIf((() -> !everythingIsZeroed())));

        NamedCommands.registerCommand("Shoot",
                Commands.waitUntil(this::everythingIsZeroed)
                        .andThen(this::enableShootModeInAuto)
                        .andThen(new ShootCommand()).onlyIf(indexer::isCollected).andThen(this::shot));

        NamedCommands.registerCommand("DontUseLookupTable", Commands.runOnce(() -> useLookupTable = false));

        NamedCommands.registerCommand("EnableShootMode",
                Commands.race(
                    Commands.waitUntil(indexer::isCollected).andThen(this::enableShootModeInAuto),
                    Commands.waitSeconds(0.25)
        ));



        NamedCommands.registerCommand("SpinUp", Commands.runOnce(shooter::enable).andThen(() -> logger.info("Shooter Spun Up")));

        NamedCommands.registerCommand("CollectorDown", Commands.runOnce(collector::setDeployed)
                .andThen(() -> collector.setCollectionState(Collector.CollectionState.COLLECTING)));

        NamedCommands.registerCommand("CollectorUp", Commands.runOnce(collector::setStowed));

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

        var inst = NetworkTableInstance.getDefault();
        inst.addListener(
                inst.getBooleanTopic("/FMSInfo/IsRedAlliance").getEntry(true),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> {
                    var command = autoChooser.get();
                    if (command != null)
                        this.onAutoChange(command.getName());
                    else
                        this.onAutoChange(AutoConstants.DEFAULT_AUTO_NAME);
                }
        );
    }

    private void enableShootModeInAuto() {
        if (isUsingLookupTable()) {
            Double angle = getSelectedAutoShotAngle();
            if (angle != null) {
                Shooter.getInstance().setAngle(angle);
            }
        }

        Shooter.getInstance().enableReadyMode();
    }

    public boolean isUsingLookupTable() {
        return useLookupTable;
    }

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    public Command getSelectedAuto() {
        return autoChooser.get();
    }

    public String getSelectedAutoName() {
        return currentAutoName;
    }

    public Double getSelectedAutoShotAngle() {
        if (shotNumber >= currentAutoAngles.length) return null;
        return Units.degreesToRotations(currentAutoAngles[shotNumber]);
    }

    public void shot() {
        if (useLookupTable && RobotState.isAutonomous())
            shotNumber++;
    }

    boolean everythingIsZeroed() {
        return Collector.getInstance().isZeroed() && Shooter.getInstance().isZeroed();
    }

    public void resetAuto() {
        Indexer.getInstance().setState(Indexer.State.COLLECTED);
        shotNumber = 0;
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
        shotNumber = 0;
        currentAutoName = autoName;
        currentAutoAngles = AutoConstants.SHOT_TABLE.getOrDefault(currentAutoName, new double[]{});
        if (currentAutoAngles.length == 0) {
            useLookupTable = false;
        }

        new InstantCommand(() ->
            postAutoTrajectory(chassis.getField(), autoName)).schedule();
    }

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }
}
