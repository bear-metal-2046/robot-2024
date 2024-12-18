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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.commands.AlignSwerveCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.vision.ATVision;
import org.tahomarobotics.robot.vision.VisionConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.tahomarobotics.robot.chassis.ChassisConstants.ACCELERATION_LIMIT;
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
    private final NapCorrectingKinematics kinematics;

    private final CalibrationData<Double[]> swerveCalibration;

//    private final ObjectDetectionCamera objectDetectionCamera;
    private final List<ATVision> apriltagCameras = new ArrayList<>();

    private final Thread odometryThread;

    private boolean isFieldOriented = true;

    private final PIDController shootModeController;

    private double targetShootingAngle;

    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds(), currentAcceleration = new ChassisSpeeds();
    private final LinearFilter xAccelFilter = LinearFilter.movingAverage(3),
            yAccelFilter = LinearFilter.movingAverage(3),
            omegaAccelFilter = LinearFilter.movingAverage(3);

    private double energyUsed = 0;

    private double totalCurrent = 0;

    private Rotation2d yaw = new Rotation2d();

    private SwerveModulePosition lastModulePosition[];

    private final SwerveDriveLimiter accelerationLimiter;
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

        lastModulePosition = getSwerveModulePositions();

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

        accelerationLimiter = new SwerveDriveLimiter(getSwerveModuleStates(), ACCELERATION_LIMIT);

        shootModeController = ChassisConstants.SHOOT_MODE_CONTROLLER;
        shootModeController.enableContinuousInput(0, 2 * Math.PI);

        odometryThread = new Thread(Robot.isReal() ? this::odometryThread : this::simulatedOdometryThread);
        odometryThread.start();


//        objectDetectionCamera = new ObjectDetectionCamera(VisionConstants.Camera.COLLECTOR_LEFT);
        apriltagCameras.add(new ATVision(VisionConstants.Camera.COLLECTOR_RIGHT, fieldPose, poseEstimator));
        apriltagCameras.add(new ATVision(VisionConstants.Camera.SHOOTER_LEFT, fieldPose, poseEstimator));
        apriltagCameras.add(new ATVision(VisionConstants.Camera.SHOOTER_RIGHT, fieldPose, poseEstimator));
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
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
        return yaw;
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

    private SwerveModuleState[] getSwerveModuleAccelerationStates() {
        return modules.stream().map(SwerveModule::getAccelerationState).toArray(SwerveModuleState[]::new);
    }

    private SwerveModuleState[] getSwerveModuleRawAccelerationStates() {
        return modules.stream().map(SwerveModule::getRawAccelerationState).toArray(SwerveModuleState[]::new);
    }

    private SwerveModuleState[] getSwerveModuleDesiredStates() {
        return modules.stream().map(SwerveModule::getDesiredState).toArray(SwerveModuleState[]::new);
    }

    public List<SwerveModule> getSwerveModules() {
        return modules;
    }

    public List<ATVision> getApriltagCameras() {
        return apriltagCameras;
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return currentChassisSpeeds;
    }

    public ChassisSpeeds getCurrentAcceleration() {
        return currentAcceleration;
    }

    // PERIODIC

    @Override
    public void periodic() {
        modules.forEach(SwerveModule::periodic);
        Pose2d pose = getPose();

        apriltagCameras.forEach(ATVision::update);

        ChassisSpeeds newChassisSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        ChassisSpeeds unfilteredAcceleration = newChassisSpeeds.minus(currentChassisSpeeds).div(Robot.defaultPeriodSecs);
        currentAcceleration = new ChassisSpeeds(
                xAccelFilter.calculate(ChassisConstants.clampAccel(unfilteredAcceleration.vxMetersPerSecond)),
                yAccelFilter.calculate(ChassisConstants.clampAccel(unfilteredAcceleration.vyMetersPerSecond)),
                omegaAccelFilter.calculate(ChassisConstants.clampAccel(unfilteredAcceleration.omegaRadiansPerSecond))
        );
        currentChassisSpeeds = newChassisSpeeds;

        double voltage = RobotController.getBatteryVoltage();
        totalCurrent = modules.stream().mapToDouble(SwerveModule::getTotalCurent).sum();
        energyUsed += totalCurrent * voltage * Robot.defaultPeriodSecs;

        SafeAKitLogger.recordOutput("Chassis/States", getSwerveModuleStates());
        SafeAKitLogger.recordOutput("Chassis/DesiredState", getSwerveModuleDesiredStates());
        SafeAKitLogger.recordOutput("Chassis/Speed", getCurrentChassisSpeeds());
        SafeAKitLogger.recordOutput("Chassis/Acceleration", getCurrentAcceleration());
        SafeAKitLogger.recordOutput("Chassis/Gyro/Yaw", getYaw());
        SafeAKitLogger.recordOutput("Chassis/Pose", pose);
        SafeAKitLogger.recordOutput("Chassis/IsAtShootingAngle?", isReadyToShoot());
        SafeAKitLogger.recordOutput("Chassis/TargetShootingAngle", targetShootingAngle);

        SafeAKitLogger.recordOutput("Chassis/Energy", getEnergyUsed());
        SafeAKitLogger.recordOutput("Chassis/TotalCurrent", totalCurrent);

        fieldPose.setRobotPose(pose);
        SmartDashboard.putData(fieldPose);

        if (RobotState.isEnabled()) {

            if (Shooter.getInstance().inReadyMode()) {
                aimToTarget(desiredSpeeds, (Shooter.getInstance().inPassingMode()) ? PASS_TARGET_POSITION.get() : SPEAKER_TARGET_POSITION.get());
            }

            var swerveModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY);

            swerveModuleStates = accelerationLimiter.calculate(swerveModuleStates);

            setSwerveStates(swerveModuleStates);
        }
    }

    private SwerveModuleState[] limitAcceleration(SwerveModuleState inputs[]) {
        double average = 0;
        for (SwerveModuleState input : inputs) {
            average += input.speedMetersPerSecond;

        }

        return null;
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

    private void aimToTarget(ChassisSpeeds speeds, Translation2d target) {

        //
        // Based off of 2022 Cheesy Poof shooting utils
        // https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/frc2022/shooting/ShootingUtil.java#L26
        //

        var pose = getPose();

        // Get polar coordinates (theta + distance) from robot to goal
        var robotToGoal = target.minus(pose.getTranslation());
        var goalRot = MathUtil.angleModulus(robotToGoal.getAngle().getRadians() + Math.PI);

        if (Shooter.getInstance().inReadyMode()) {
            if (Shooter.getInstance().inPassingMode()) {
                Shooter.getInstance().angleToPass();
            } else {
                Shooter.getInstance().angleToSpeaker();
            }
        }

        targetShootingAngle = goalRot - Units.degreesToRadians(4);

        fieldPose.getObject("goal").setPose(new Pose2d(target, new Rotation2d()));

        speeds.omegaRadiansPerSecond =
                shootModeController.calculate(
                        pose.getRotation().getRadians(),
                        targetShootingAngle
                );
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
            while (true) {
                Thread.sleep(4);
                updatePosition();
            }
        } catch (InterruptedException e) {
            logger.warn("Simulated odometry thread sleep", e);
            Thread.currentThread().interrupt();
        }
    }

    private SwerveModulePosition[] calculateModuleDeltas(SwerveModulePosition[] start, SwerveModulePosition[] end) {
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[end.length];
        for (int i = 0; i < end.length; i++) {
            moduleDeltas[i] = new SwerveModulePosition(end[i].distanceMeters - start[i].distanceMeters, end[i].angle);
        }

        return moduleDeltas;
    }

    private void updatePosition() {

        SwerveModulePosition[] modulePositions;
        // Calculate new position

        synchronized (modules) {
            modulePositions = getSwerveModulePositions();
        }

        synchronized (gyroIO) {

            var validYaw = gyroIO.getYaw();
            if (validYaw.valid()) {
                yaw = validYaw.yaw();
            } else {
                // Use the angle delta from the kinematics and module deltas
                SwerveModulePosition[] deltas = calculateModuleDeltas(lastModulePosition, modulePositions);
                Twist2d twist = kinematics.toTwist2d_super(deltas);
                yaw = yaw.plus(new Rotation2d(twist.dtheta));
            }
        }

        lastModulePosition = Arrays.copyOf(modulePositions, modulePositions.length);

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
