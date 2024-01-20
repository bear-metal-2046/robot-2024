package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import java.util.List;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

public class SwerveModuleIO {
    private final org.slf4j.Logger logger;

    // MEMBER VARIABLES

    protected String name;

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerAbsEncoder;

    protected SwerveModuleState desiredState = new SwerveModuleState();

    private double angularOffset;

    private final StatusSignal<Double> steerPosition;
    private final StatusSignal<Double> steerVelocity;
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0);

    @AutoLog
    static class SwerveModuleIOInputs {
        public SwerveModuleState desiredState = new SwerveModuleState();
    }

    // CONSTRUCTOR

    public SwerveModuleIO(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        logger = LoggerFactory.getLogger("SwerveModule." + descriptor.moduleName());
        name = "Chassis/Modules/" + descriptor.moduleName();

        this.angularOffset = angularOffset;

        driveMotor = new TalonFX(descriptor.driveId(), RobotConfiguration.CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), RobotConfiguration.CANBUS_NAME);
        steerAbsEncoder = new CANcoder(descriptor.encoderId(), RobotConfiguration.CANBUS_NAME);

        configureDriveMotor(driveMotor.getConfigurator());
        configureSteerMotor(steerMotor.getConfigurator(), descriptor.encoderId());
        configureEncoder(steerAbsEncoder.getConfigurator(), angularOffset);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = steerAbsEncoder.getAbsolutePosition();
        steerVelocity = steerAbsEncoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY,
                drivePosition, driveVelocity, steerPosition,
                steerVelocity
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerAbsEncoder);

        driveMotorVelocity.EnableFOC = RobotConfiguration.USING_PHOENIX_PRO;
        steerMotorPosition.EnableFOC = RobotConfiguration.USING_PHOENIX_PRO;
    }

    // CALIBRATION

    
    public void initializeCalibration() {
        applyAngularOffset(0);
        steerMotor.setControl(new CoastOut());
    }

    
    public double finalizeCalibration() {
        angularOffset = -steerPosition.refresh().getValue();
        applyAngularOffset(angularOffset);
        steerMotor.setControl(new StaticBrake());
        return angularOffset;
    }

    
    public void cancelCalibration() {
        applyAngularOffset(angularOffset);
    }

    private void applyAngularOffset(double offset) {
        var config = new MagnetSensorConfigs();
        config.MagnetOffset = offset;
        if (steerAbsEncoder.getConfigurator().apply(config) != StatusCode.OK) {
            logger.error("Failed to apply angular offset to " + offset);
        }
    }

    // Getters

    /**
     * @return Position of the module in <strong>meters</strong>.
     */
    public double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(drivePosition.refresh(), driveVelocity.refresh()) * DRIVE_POSITION_COEFFICIENT;
    }

    /**
     * @return Velocity of the module in <strong>meters/s</strong>.
     */
    public double getDriveVelocity() {
        return driveVelocity.refresh().getValue() * DRIVE_POSITION_COEFFICIENT;
    }

    /**
     * @return Relative rotation of the module in <strong>rotations</strong>.
     */
    public double getSteerAngle() {
        return BaseStatusSignal.getLatencyCompensatedValue(steerPosition.refresh(), steerVelocity.refresh());
    }

    /**
     * @return The current state of the module.
     */
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(getSteerAngle()));
    }

    
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * @return The current position of the module.
     */
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(getSteerAngle()));
    }

    // State

    
    public void processInputs(SwerveModuleIOInputs inputs) {
        desiredState = inputs.desiredState;
    }

    
    public void periodic() {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                steerPosition,
                steerVelocity
        );

        Logger.recordOutput(name + "/State", getState());
        Logger.recordOutput(name + "/DesiredState", desiredState);
        Logger.recordOutput(name + "/Position", getPosition());

        Logger.recordOutput(name + "/DriveVelocity", driveVelocity.getValueAsDouble());
        Logger.recordOutput(name + "/DriveVelocityMPS", driveVelocity.getValueAsDouble() * DRIVE_POSITION_COEFFICIENT);
        Logger.recordOutput(name + "/SteerVelocity", steerVelocity.getValueAsDouble());
    }

    
    public void updateDesiredState() {
        double steerAngle = getSteerAngle();

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(steerAngle));

        desiredState.angle = Rotation2d.fromRotations((desiredState.angle.getRotations() % 1.0 + 1.0) % 1.0);

        driveMotor.setControl(driveMotorVelocity.withVelocity(desiredState.speedMetersPerSecond / DRIVE_POSITION_COEFFICIENT));
        steerMotor.setControl(steerMotorPosition.withPosition(desiredState.angle.getRotations()));
    }

    
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public List<BaseStatusSignal> getStatusSignals() {
        return List.of(
                drivePosition,
                driveVelocity,
                steerPosition,
                steerVelocity
        );
    }
}
