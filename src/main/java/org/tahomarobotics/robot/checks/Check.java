package org.tahomarobotics.robot.checks;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.amp.commands.SourceIntakeCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.chassis.SwerveModule;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;
import org.tahomarobotics.robot.climbers.commands.UnladenClimbCommand;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.shooter.commands.ShootCommand;
import org.tahomarobotics.robot.util.SafeAKitLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

public class Check extends ParallelCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(Check.class);

    public Check() {
        addCommands(
                Commands.runOnce(() -> logger.info("-----STARTING SYSTEMS CHECK-----")),
                Commands.sequence(
                        checkAllModuleSteers(),
                        checkAllModuleDrives()
                ),
                checkClimbersAndShooter(),
                checkCollectorAndIndexer(),
                checkAmpArm()
        );
    }

    public static MotionMagicVelocityVoltage steerControlRequest = new MotionMagicVelocityVoltage(5);
    public static MotionMagicVelocityVoltage driveControlRequest = new MotionMagicVelocityVoltage(10 / ChassisConstants.DRIVE_REDUCTION);

    public static Command checkModuleSteer(SwerveModule module) {
        return Check.runForTime(logger, module.name + " Module/Steer", 4, module::testSteer, module::stop, module::getSteerCurrent, new Check.NominalValues(1.75, 3.52));
    }

    public static Command checkModuleDrive(SwerveModule module) {
        return Check.runForTime(logger, module.name + " Module/Drive", 4, module::testDrive, module::stop, module::getDriveCurrent, new Check.NominalValues(.45, 2.0));
    }

    public static Command checkAllModuleSteers() {
        Command cmd = Commands.parallel(Chassis.getInstance().getSwerveModules().stream().map(Check::checkModuleSteer).toArray(Command[]::new));
        cmd.addRequirements(Chassis.getInstance());
        return cmd;
    }

    public static Command checkAllModuleDrives() {
        Command cmd = Commands.parallel(Chassis.getInstance().getSwerveModules().stream().map(Check::checkModuleDrive).toArray(Command[]::new));
        cmd.addRequirements(Chassis.getInstance());
        return cmd;
    }

    static {
        SmartDashboard.putData("Check Swerve Steers", checkAllModuleSteers());
        SmartDashboard.putData("Check Swerve Drives", checkAllModuleDrives());
    }

    static Command checkClimbersAndShooter() {
        Climbers climbers = Climbers.getInstance();
        Shooter shooter = Shooter.getInstance();

        List<Double> leftCurrents = new ArrayList<>();
        List<Double> rightCurrents = new ArrayList<>();
        List<Double> pivotCurrents = new ArrayList<>();
        List<Double> shooterLeftCurrents = new ArrayList<>();
        List<Double> shooterRightCurrents = new ArrayList<>();

        Command command = Commands.sequence(
                Commands.runOnce(() -> {
                    leftCurrents.clear();
                    rightCurrents.clear();
                    pivotCurrents.clear();
                    shooterLeftCurrents.clear();
                    shooterRightCurrents.clear();
                }),
                Commands.sequence(
                        Commands.runOnce(() -> shooter.setAngle(ShooterConstants.MAX_PIVOT_ANGLE)),
                        Commands.waitUntil(shooter::isAtAngle),
                        Commands.runOnce(shooter::idle),
                        new UnladenClimbCommand(ClimberConstants.PARTIAL_CLIMB_POSITION),
                        new UnladenClimbCommand(ClimberConstants.BOTTOM_POSITION),
                        Commands.runOnce(shooter::stop),
                        Commands.runOnce(() -> shooter.setAngle(ShooterConstants.MIN_PIVOT_ANGLE))
                ).raceWith(Commands.run(() -> {
                    leftCurrents.add(climbers.getLeftCurrent());
                    rightCurrents.add(climbers.getRightCurrent());
                    pivotCurrents.add(shooter.getPivotCurrent());
                    shooterLeftCurrents.add(shooter.getBottomShooterCurrent());
                    shooterRightCurrents.add(shooter.getTopShooterCurrent());
                })),
                Commands.runOnce(() -> {
                    Check.printStatistics(logger, "Left Climber", leftCurrents, new Check.NominalValues(1.5, 3.0));
                    Check.printStatistics(logger, "Right Climber", rightCurrents, new Check.NominalValues(2.15, 3.15));
                    Check.printStatistics(logger, "Shooter Pivot", pivotCurrents, new Check.NominalValues(.15, 1.0));
                    Check.printStatistics(logger, "Shooter Left", shooterLeftCurrents, new Check.NominalValues(6.25, 8.0));
                    Check.printStatistics(logger, "Shooter Right", shooterRightCurrents, new Check.NominalValues(7.25, 9.0));
                })
        );

        command.addRequirements(climbers, shooter);
        return command;
    }

    static {
        SmartDashboard.putData("Check Climbers and Shooter Pivot", checkClimbersAndShooter());
    }


    static Command checkCollectorAndIndexer() {
        Collector collector = Collector.getInstance();
        Indexer indexer = Indexer.getInstance();

        List<Double> leftCurrents = new ArrayList<>();
        List<Double> rightCurrents = new ArrayList<>();
        List<Double> spinyCurrents = new ArrayList<>();
        List<Double> indexerCurrents = new ArrayList<>();

        Command command = Commands.sequence(
                Commands.runOnce(() -> {
                    leftCurrents.clear();
                    rightCurrents.clear();
                    spinyCurrents.clear();
                    indexerCurrents.clear();
                }),
                Commands.sequence(
                        Commands.runOnce(collector::setDeployed),
                        Commands.runOnce(() -> collector.setIsCollecting(true)),
                        Commands.waitSeconds(2),
                        Commands.runOnce(collector::setStowed),
                        Commands.runOnce(() -> collector.setIsCollecting(false))
                ).raceWith(Commands.run(() -> {
                    leftCurrents.add(collector.getLeftDeployCurrent());
                    rightCurrents.add(collector.getRightDeployCurrent());
                    spinyCurrents.add(collector.getSpinyCurrent());
                    indexerCurrents.add(indexer.getCurrent());
                })),
                Commands.runOnce(() -> {
                    Check.printStatistics(logger, "Left Collector", leftCurrents, new Check.NominalValues(0.15, 0.75));
                    Check.printStatistics(logger, "Right Collector", rightCurrents, new Check.NominalValues(0.15, 0.75));
                    Check.printStatistics(logger, "Collect Collector", spinyCurrents, new Check.NominalValues(15, 25));
                    Check.printStatistics(logger, "Indexer", indexerCurrents, new Check.NominalValues(2.0, 3.25));
                })
        );

        command.addRequirements(collector, indexer);
        return command;
    }

    static {
        SmartDashboard.putData("Check Collector and Indexer", checkCollectorAndIndexer());
    }

    static Command checkAmpArm() {
        AmpArm arm = AmpArm.getInstance();

        List<Double> armCurrents = new ArrayList<>();
        List<Double> wristCurrents = new ArrayList<>();
        List<Double> rollerCurrents = new ArrayList<>();

        Command command = Commands.sequence(
                Commands.runOnce(() -> {
                    armCurrents.clear();
                    wristCurrents.clear();
                    rollerCurrents.clear();
                }),
                Commands.sequence(
                        Commands.runOnce(() -> {
                            arm.setArmPosition(.25);
                            arm.setWristPosition(5);
                            arm.setRollerState(AmpArm.RollerState.PASSING);
                        }),
                        Commands.waitUntil(arm::isArmAtPosition),
                        Commands.waitUntil(arm::isWristAtPosition),
                        Commands.runOnce(() -> arm.setArmState(AmpArm.ArmState.STOW)),
                        Commands.waitUntil(arm::isArmAtPosition),
                        Commands.waitUntil(arm::isWristAtPosition),
                        Commands.runOnce(() -> arm.setRollerState(AmpArm.RollerState.DISABLED))
                ).raceWith(Commands.run(() -> {
                    armCurrents.add(arm.getArmCurrent());
                    wristCurrents.add(arm.getWristCurrent());
                    rollerCurrents.add(arm.getRollerCurrent());
                })),
                Commands.runOnce(() -> {
                    Check.printStatistics(logger, "Arm", armCurrents, new Check.NominalValues(1.5, 2.5));
                    Check.printStatistics(logger, "Wrist", wristCurrents, new Check.NominalValues(0.5, 3.75));
                    Check.printStatistics(logger, "Rollers", rollerCurrents, new Check.NominalValues(9.5, 12));
                })
        );

        command.addRequirements(arm);
        return command;
    }

    static {
        SmartDashboard.putData("Check Amp Arm", checkAmpArm());
    }

    static Command checkVision() {
        Chassis chassis = Chassis.getInstance();
        ParallelCommandGroup cmd = new ParallelCommandGroup();

        chassis.getApriltagCameras().forEach(c -> {
            cmd.addCommands(
                Commands.sequence(
                    Commands.runOnce(() -> {
                        logger.info("Waiting to see an apriltag on " + c.getName() + "...");
                        SafeAKitLogger.recordOutput("Checks/ATVision/" + c.getName(), false);
                    }),
                    Commands.waitUntil(() -> c.aprilTagCount() == 1),
                    Commands.runOnce(() -> {
                        logger.info(c.getName() + " saw a single apriltag!");
                        SafeAKitLogger.recordOutput("Checks/ATVision/" + c.getName(), true);
                    })

            ));
        });

        return cmd.ignoringDisable(true);
    }

    static {
        SmartDashboard.putData("Check Cameras", checkVision());
    }

    static Command runForTime(Logger logger, String name, double time, Runnable run, Runnable stop, Supplier<Double> current, NominalValues nom) {
        List<Double> currents = new ArrayList<>();
        return Commands.sequence(
                Commands.sequence(
                        Commands.runOnce(currents::clear),
                        Commands.runOnce(run),
                        Commands.waitSeconds(time),
                        Commands.runOnce(stop)
                ).raceWith(Commands.run(() -> currents.add(current.get()))),
                Commands.runOnce(() -> printStatistics(logger, name, currents, nom))
        );
    }

    static void printStatistics(Logger logger, String name, List<Double> currents, NominalValues nom) {
        Supplier<DoubleStream> _currents = () -> currents.stream().mapToDouble(d -> d).filter(d -> d >= 0);
        double avg = _currents.get().average().orElse(0);
        double max = _currents.get().max().orElse(0);

        if (avg < nom.minAverage() || avg > nom.maxAverage()) {
            logger.error(name + " is not nominal!\nThe average was: " + avg + ", which is out of the [" + nom.minAverage + ", " + nom.maxAverage + "].");
            SafeAKitLogger.recordOutput("Checks/" + name + "/Good", false);
        } else {
            SafeAKitLogger.recordOutput("Checks/" + name + "/Good", true);
        }

        SafeAKitLogger.recordOutput("Checks/" + name + "/Avg", avg);
        SafeAKitLogger.recordOutput("Checks/" + name + "/Max", max);
    }

    public static void register() {
        SmartDashboard.putData("Check", new Check());
        SmartDashboard.putData("Check System", checkSystem());
    }

    public record NominalValues(double minAverage, double maxAverage) {
    }


    static Command checkSystem() {
        Collector collector = Collector.getInstance();
        Indexer indexer = Indexer.getInstance();
        Shooter shooter = Shooter.getInstance();
        AmpArm arm = AmpArm.getInstance();

        var cmd = Commands.sequence(
                Commands.runOnce(collector::toggleDeploy),
                Commands.runOnce(() -> collector.setCollectionState(Collector.CollectionState.COLLECTING)),
                Commands.waitUntil(indexer::isCollected),
                Commands.runOnce(shooter::toggleRedundantShootModeFar),
                new ShootCommand(),
                Commands.runOnce(collector::toggleDeploy),
                Commands.runOnce(() -> collector.setCollectionState(Collector.CollectionState.COLLECTING)),
                Commands.waitUntil(indexer::isCollected),
                AmpArmCommands.FEEDFORWARD.get(),
                AmpArmCommands.ARM_TO_AMP.get(),
                Commands.runOnce(() -> arm.setRollerState(AmpArm.RollerState.SCORE)),
                AmpArmCommands.ARM_TO_STOW.get(),
                AmpArmCommands.ARM_TO_SOURCE.get(),
                new SourceIntakeCommand(),
                AmpArmCommands.ARM_TO_PASSTHROUGH.get(),
                AmpArmCommands.FEEDBACK.get()
        );
        cmd.addRequirements(collector, shooter, indexer, arm);

        return cmd;
    }
}
