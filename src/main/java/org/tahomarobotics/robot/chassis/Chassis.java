package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.commands.AlignSwerveCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.ToggledOutputs;
import org.tahomarobotics.robot.vision.ATVision;
import org.tahomarobotics.robot.vision.VisionConstants;

import java.util.ArrayList;
import java.util.List;

import static org.tahomarobotics.robot.shooter.ShooterConstants.SPEAKER_TARGET_POSITION;

public class Chassis extends SubsystemIF implements ToggledOutputs {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(Chassis.class);
    private static final Chassis INSTANCE = new Chassis();

    private final GyroIO gyroIO = RobotConfiguration.getMode() == RobotConfiguration.Mode.REAL ? new GyroIO() : new GyroIOSim();

    // Member Variables
    private final List<SwerveModule> modules;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d fieldPose = new Field2d();
    private final SwerveDriveKinematics kinematics;

    private final CalibrationData<Double[]> swerveCalibration;

    private final ATVision collectorLeftVision;
    private final ATVision collectorRightVision;
    private final ATVision shooterLeftVision;
    private final ATVision shooterRightVision;

    private final Thread odometryThread;

    private boolean isFieldOriented = true;

    private final PIDController shootModeController;

    // CONSTRUCTOR

    private Chassis() {
        // Read the calibration data from the roboRIO.
        swerveCalibration = new CalibrationData<>("SwerveCalibration", new Double[]{0d, 0d, 0d, 0d});

        // Use the calibration to create the SwerveModules.
        Double[] angularOffsets = swerveCalibration.get();
        modules = List.of(
                new SwerveModule(RobotMap.FRONT_LEFT_MOD, angularOffsets[0]),
                new SwerveModule(RobotMap.FRONT_RIGHT_MOD, angularOffsets[1]),
                new SwerveModule(RobotMap.BACK_LEFT_MOD, angularOffsets[2]),
                new SwerveModule(RobotMap.BACK_RIGHT_MOD, angularOffsets[3])
        );

        kinematics = new SwerveDriveKinematics(
                modules.stream()
                    .map(SwerveModule::getTranslationOffset)
                    .toArray(Translation2d[]::new)
        );


        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                getSwerveModulePositions(),
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                VecBuilder.fill(0.02, 0.02, 0.02),
                VecBuilder.fill(0.1, 0.1, 0.01)
        );

        shootModeController = ChassisConstants.SHOOT_MODE_CONTROLLER;
        shootModeController.enableContinuousInput(0, 2 * Math.PI);

        odometryThread = new Thread(Robot.isReal() ? this::odometryThread : this::simulatedOdometryThread);
        odometryThread.start();


        collectorLeftVision = new ATVision(VisionConstants.ATCamera.COLLECTOR_LEFT, fieldPose, poseEstimator);
        collectorRightVision = new ATVision(VisionConstants.ATCamera.COLLECTOR_RIGHT, fieldPose, poseEstimator);
        shooterLeftVision = new ATVision(VisionConstants.ATCamera.SHOOTER_LEFT, fieldPose, poseEstimator);
        shooterRightVision = new ATVision(VisionConstants.ATCamera.SHOOTER_RIGHT, fieldPose, poseEstimator);
    }

    public static Chassis getInstance() {
        return INSTANCE;
    }

    public Field2d getField() {
        return fieldPose;
    }

    // INITIALIZE

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putData("Align Swerves", new AlignSwerveCommand());

        gyroIO.zero();

        var gyro = getYaw();
        var modules = getSwerveModulePositions();
        synchronized (poseEstimator) {
            poseEstimator.resetPosition(gyro, modules, new Pose2d());
        }

        return this;
    }

    // Calibration

    public void initializeCalibration() {
        modules.forEach(SwerveModule::initializeCalibration);
    }

    public void finalizeCalibration() {
        swerveCalibration.set(
                modules.stream()
                        .map(SwerveModule::finalizeCalibration)
                        .toArray(Double[]::new)
        );
    }

    public void cancelCalibration() {
        modules.forEach(SwerveModule::cancelCalibration);
    }

    // Getters

    public Rotation2d getYaw() {
        return gyroIO.getYaw();
    }

    public Pose2d getPose() {
        synchronized (poseEstimator) {
            return poseEstimator.getEstimatedPosition();
        }
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    private SwerveModuleState[] getSwerveModuleStates() {
        return modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    private SwerveModuleState[] getSwerveModuleDesiredStates() {
        return modules.stream().map(SwerveModule::getDesiredState).toArray(SwerveModuleState[]::new);
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    // PERIODIC

    @Override
    public void periodic() {
        modules.forEach(SwerveModule::periodic);
        Pose2d pose = getPose();

        recordOutput("Chassis/States", getSwerveModuleStates());
        recordOutput("Chassis/DesiredState", getCurrentChassisSpeeds());
        recordOutput("Chassis/CurrentChassisSpeeds", getCurrentChassisSpeeds());
        recordOutput("Chassis/Gyro/Yaw", getYaw());
        recordOutput("Chassis/Pose", pose);

        fieldPose.setRobotPose(pose);
        SmartDashboard.putData(fieldPose);
    }

    @Override
    public void simulationPeriodic() {
        ((GyroIOSim) gyroIO).simulationPeriodic(getCurrentChassisSpeeds());
    }

    // STATE
    public void drive(ChassisSpeeds velocity) {
        if (!isFieldOriented && DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            velocity = new ChassisSpeeds(-velocity.vxMetersPerSecond, -velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond);
        }

        drive(velocity, isFieldOriented);
    }

    public void drive(ChassisSpeeds velocity, boolean fieldRelative) {
        if (fieldRelative) {
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getPose().getRotation());
        }

        if (Shooter.getInstance().inShootingMode()) {
            aimToSpeaker(velocity);
        }

        velocity = ChassisSpeeds.discretize(velocity, Robot.defaultPeriodSecs);

        var swerveModuleStates = kinematics.toSwerveModuleStates(velocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY);
        setSwerveStates(swerveModuleStates);
    }

    public void autoDrive(ChassisSpeeds velocity) {
        velocity = ChassisSpeeds.discretize(velocity, Robot.defaultPeriodSecs);

        if (Shooter.getInstance().inShootingMode()) {
            aimToSpeaker(velocity);
        }

        var swerveModuleStates = kinematics.toSwerveModuleStates(velocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY);
        setSwerveStates(swerveModuleStates);
    }

    // SETTERS

    private void aimToSpeaker(ChassisSpeeds speeds) {

        //
        // Based off of 2022 Cheesy Poof shooting utils
        // https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java#L26
        //

        var pose = getPose();
        var goal = SPEAKER_TARGET_POSITION.get();

        // Get polar coordinates (theta + distance) from robot to goal
        var robotToGoal = goal.minus(pose.getTranslation());
        var goalRot = MathUtil.angleModulus(robotToGoal.getAngle().getRadians() + Math.PI);
        var goalDis = robotToGoal.getNorm();

        // Get robot speed relative to angle from robot to goal
        // X component is towards/from goal, Y component is tangential to goal
        var curSpeeds = getCurrentChassisSpeeds();
        var speedsTranslation = new Translation2d(curSpeeds.vxMetersPerSecond, curSpeeds.vyMetersPerSecond);

        var speedsToGoal = speedsTranslation.rotateBy(robotToGoal.getAngle().unaryMinus());

        double tangentialComponent = speedsToGoal.getY();
        double radialComponent = speedsToGoal.getX();

        // Shooter angle speed compensation
        Shooter.getInstance().angleToSpeaker(radialComponent);

        // Calculate position and velocity adjustment
        double adj = Math.atan2(-tangentialComponent, ShooterConstants.SHOT_SPEED + radialComponent);
        double adjSpeed = tangentialComponent / goalDis;

        // modifiers
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            adj *= -1;
            adjSpeed *= -1;
        }

        double diff = pose.getRotation().getRadians() - (goalRot + adj);
        recordOutput("Chassis/Angle Difference", diff);
        recordOutput("Chassis/Adjustment", adj);
        recordOutput("Chassis/Adjustment Speed", adjSpeed);

        fieldPose.getObject("goal").setPose(new Pose2d(goal, new Rotation2d()));

        speeds.omegaRadiansPerSecond =
                shootModeController.calculate(
                        pose.getRotation().getRadians(),
                        goalRot + adj
                ) - adjSpeed;

        recordOutput("Chassis/Angular Velocity Output", speeds.omegaRadiansPerSecond);
    }

    private void setSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) modules.get(i).setDesiredState(states[i]);
    }

    public void resetOdometry(Pose2d pose) {
        var gyro = getYaw();
        var modules = getSwerveModulePositions();
        synchronized (poseEstimator) {
            poseEstimator.resetPosition(gyro, modules, pose);
        }
        logger.info("Reset Pose: " + pose);
    }

    public void toggleOrientation() {
        isFieldOriented = !isFieldOriented;
    }

    public void orientToZeroHeading() {
        Rotation2d heading = new Rotation2d(DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue ? 0.0 : Math.PI);
        resetOdometry(new Pose2d(getPose().getTranslation(), heading));
    }

    // onInit

    @Override
    public void onDisabledInit() {
        modules.forEach(SwerveModule::stop);
    }



    // Odometry Thread

    /**
     * Simple thread which runs at the status period rate for all CAN devices
     * in chassis.
     */
    private void odometryThread() {


        Threads.setCurrentThreadPriority(true, 1);

        // Get signals array
        List<BaseStatusSignal> signalList = new ArrayList<>(gyroIO.getStatusSignals());
        for (var module : this.modules) {
            signalList.addAll(module.getStatusSignals());
        }

        BaseStatusSignal[] signals = signalList.toArray(BaseStatusSignal[]::new);



        while (true) {

            // Wait for all signals to arrive
            StatusCode status = BaseStatusSignal.waitForAll(6 / RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY, signals);

            if (status.isError()) logger.error("Failed to waitForAll updates" + status.getDescription());

            updatePosition();
        }
    }

    private void simulatedOdometryThread() {
        try {
            while(true) {
                Thread.sleep(4);
                updatePosition();
            }
        } catch (InterruptedException e) {
            logger.warn("Simulated odometry thread sleep", e);
            Thread.currentThread().interrupt();
        }
    }

    private void updatePosition() {
        Rotation2d yaw;
        SwerveModulePosition[] modulePositions;
        // Calculate new position

        synchronized (gyroIO) {
            yaw = getYaw();
        }

        synchronized (modules) {
            modulePositions = getSwerveModulePositions();
        }

        synchronized (poseEstimator) {
            poseEstimator.update(yaw, modulePositions);
        }
    }

    @Override
    public boolean logOutputs() {
        return OutputsConfiguration.CHASSIS;
    }
}
