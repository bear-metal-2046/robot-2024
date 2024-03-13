package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.commands.AlignSwerveCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.vision.ATVision;
import org.tahomarobotics.robot.vision.ObjectDetectionCamera;
import org.tahomarobotics.robot.vision.VisionConstants;

import java.util.ArrayList;
import java.util.List;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class Chassis extends SubsystemIF {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(Chassis.class);
    private static final Chassis INSTANCE = new Chassis();

    private final GyroIO gyroIO = RobotConfiguration.getMode() == RobotConfiguration.Mode.REAL ? new GyroIO() : new GyroIOSim();

    // Member Variables
    private final List<SwerveModule> modules;

    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d fieldPose = new Field2d();
    private final SwerveDriveKinematics kinematics;

    private final CalibrationData<Double[]> swerveCalibration;

    private final ObjectDetectionCamera objectDetectionCamera;
    private final ATVision[] apriltagCameras = new ATVision[3];

    private final Thread odometryThread;

    private boolean isFieldOriented = true;

    private final PIDController shootModeController;

    private double targetShootingAngle;

    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

    private double energyUsed = 0;

    private double totalCurrent = 0;

    private final LinearFilter lastAccelAverage = LinearFilter.movingAverage(5);
    private double lastVelocity = 0.0;

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

        kinematics = new NapCorrectingKinematics(
                this::getYaw,
                this::getPose,
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


        objectDetectionCamera
                = new ObjectDetectionCamera(VisionConstants.Camera.COLLECTOR_LEFT);
        apriltagCameras[0] = new ATVision(VisionConstants.Camera.COLLECTOR_RIGHT, fieldPose, poseEstimator);
        apriltagCameras[1] = new ATVision(VisionConstants.Camera.SHOOTER_LEFT, fieldPose, poseEstimator);
        apriltagCameras[2] = new ATVision(VisionConstants.Camera.SHOOTER_RIGHT, fieldPose, poseEstimator);
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
        return currentChassisSpeeds;
    }

    // PERIODIC

    @Override
    public void periodic() {
        modules.forEach(SwerveModule::periodic);
        Pose2d pose = getPose();

        currentChassisSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        double voltage = RobotController.getBatteryVoltage();
        totalCurrent = modules.stream().mapToDouble(SwerveModule::getTotalCurent).sum();
        energyUsed += totalCurrent * voltage * Robot.defaultPeriodSecs;

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

        double timeShotOffset = (radialComponent > 0 ? TIME_SHOT_OFFSET_POSITIVE : TIME_SHOT_OFFSET_NEGATIVE);
        double radialAcceleration = lastAccelAverage.calculate((radialComponent - lastVelocity) / 0.02);
        double radialVelocity = radialComponent + (radialAcceleration * timeShotOffset);
        lastVelocity = radialComponent;

        SafeAKitLogger.recordOutput("Chassis/RadialVelocity", radialComponent);
        SafeAKitLogger.recordOutput("Chassis/RadialAcceleration", radialAcceleration);
        SafeAKitLogger.recordOutput("Chassis/PrecomposedRadialVelocity", radialVelocity);
        SafeAKitLogger.recordOutput("Chassis/TangentialVelocity", tangentialComponent);

        SafeAKitLogger.recordOutput("Chassis/States", getSwerveModuleStates());
        SafeAKitLogger.recordOutput("Chassis/DesiredState", getSwerveModuleDesiredStates());
        SafeAKitLogger.recordOutput("Chassis/CurrentChassisSpeeds", getCurrentChassisSpeeds());
        SafeAKitLogger.recordOutput("Chassis/Gyro/Yaw", getYaw());
        SafeAKitLogger.recordOutput("Chassis/Pose", pose);
        SafeAKitLogger.recordOutput("Chassis/IsAtShootingAngle?", isReadyToShoot());
        SafeAKitLogger.recordOutput("Chassis/TargetShootingAngle", targetShootingAngle);

        SafeAKitLogger.recordOutput("Chassis/Energy", getEnergyUsed());
        SafeAKitLogger.recordOutput("Chassis/TotalCurrent", totalCurrent);

        fieldPose.setRobotPose(pose);
        SmartDashboard.putData(fieldPose);

        if (RobotState.isEnabled()) {

            if (Shooter.getInstance().inShootingMode()) {
                aimToSpeaker(desiredSpeeds);
            }

            var swerveModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY);
            setSwerveStates(swerveModuleStates);
        }

        for (ATVision c : apriltagCameras) {
            c.update();
        }
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

        desiredSpeeds = ChassisSpeeds.discretize(velocity, Robot.defaultPeriodSecs);
    }

    public void autoDrive(ChassisSpeeds velocity) {
        desiredSpeeds = ChassisSpeeds.discretize(velocity, Robot.defaultPeriodSecs);
    }

    // SETTERS

    private void aimToSpeaker(ChassisSpeeds speeds) {
        //
        // Based off of 2022 Cheesy Poof shooting utils
        // https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java#L26
        //

        // Shooter angle speed compensation
        Shooter.getInstance().angleToSpeaker(radialVelocity);

        // Calculate position and velocity adjustment
        double adj = Math.atan2(-totalCurrent, SHOT_SPEED + radialComponent);
        double adjSpeed = totalCurrent / goalDis;

        // modifiers
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            adj *= -1;
            adjSpeed *= -1;
        }

        targetShootingAngle = goalRot + adj;

        fieldPose.getObject("goal").setPose(new Pose2d(goal, new Rotation2d()));

        speeds.omegaRadiansPerSecond =
                shootModeController.calculate(
                        pose.getRotation().getRadians(),
                        targetShootingAngle
                ) - adjSpeed;
    }

    public boolean isReadyToShoot() {
        return shootModeController.atSetpoint();
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
            BaseStatusSignal.waitForAll(4 / RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY, signals);
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
    public double getEnergyUsed() {
        return energyUsed / 1000d;
    }

    @Override
    public double getTotalCurrent() {
        return totalCurrent;
    }
}
