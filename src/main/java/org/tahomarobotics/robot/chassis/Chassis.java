package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.commands.AlignSwerveCommand;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.List;

public class Chassis extends SubsystemIF {
    private static final Chassis INSTANCE = new Chassis();

    public static Chassis getInstance() {
        return INSTANCE;
    }

    // Member Variables

    private final GyroIO gyroIO = RobotConfiguration.getMode() == RobotConfiguration.Mode.REAL ? new GyroIOReal() : new GyroIOSim();

    private final List<SwerveModule> modules;

    private final SwerveDrivePoseEstimator poseEstimator;

    private boolean isFieldOriented = true;
    private final Field2d fieldPose = new Field2d();

    private final SwerveDriveKinematics kinematics;

    private final CalibrationData<Double[]> swerveCalibration;

    // Constructor

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
    }

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
        return poseEstimator.getEstimatedPosition();
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    private SwerveModuleState[] getSwerveModuleStates() {
        return modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    // State

    @Override
    public void periodic() {
        // Log outputs to the log every 20ms.
        Logger.recordOutput("Chassis/Pose", getPose());
        Logger.recordOutput("Chassis/States", getSwerveModuleStates());
        gyroIO.logOutputs();

        modules.forEach(SwerveModule::periodic);

        var gyro = getYaw();
        var modules = getSwerveModulePositions();
        synchronized (poseEstimator) {
            poseEstimator.update(gyro, modules);
        }

        fieldPose.setRobotPose(getPose());
        SmartDashboard.putData(fieldPose);

        SmartDashboard.putString("Pose", getPose().toString());
    }

    @Override
    public void simulationPeriodic() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
        ((GyroIOSim) gyroIO).simulationPeriodic(speeds);
    }

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

        velocity = ChassisSpeeds.discretize(velocity, Robot.defaultPeriodSecs);

        var swerveModuleStates = kinematics.toSwerveModuleStates(velocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY);
        setSwerveStates(swerveModuleStates);
    }

    private void setSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) modules.get(i).setDesiredState(states[i]);
    }

    private void resetOdometry(Pose2d pose) {
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
}
