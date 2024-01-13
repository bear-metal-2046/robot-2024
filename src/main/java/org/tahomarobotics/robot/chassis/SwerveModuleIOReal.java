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
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

public class SwerveModuleIOReal implements SwerveModuleIO {
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

    private final StatusSignal<Double> steerOutput;
    private final StatusSignal<Double> steerCurrentDraw;

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0);

    // CONSTRUCTOR

    public SwerveModuleIOReal(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
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

        steerOutput = steerMotor.getClosedLoopOutput();
        steerCurrentDraw = steerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY,
                drivePosition, driveVelocity, steerPosition,
                steerVelocity, steerOutput, steerCurrentDraw
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerAbsEncoder);

        driveMotorVelocity.EnableFOC = RobotConfiguration.USING_PHOENIX_PRO;
        steerMotorPosition.EnableFOC = RobotConfiguration.USING_PHOENIX_PRO;
    }

    // CALIBRATION

    @Override
    public void initializeCalibration() {
        applyAngularOffset(0);
        steerMotor.setControl(new CoastOut());
    }

    @Override
    public double finalizeCalibration() {
        angularOffset = -steerPosition.refresh().getValue();
        applyAngularOffset(angularOffset);
        steerMotor.setControl(new StaticBrake());
        return angularOffset;
    }

    @Override
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
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(getSteerAngle()));
    }

    /**
     * @return The current position of the module.
     */
    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(getSteerAngle()));
    }

    // State

    @Override
    public void processInputs(SwerveModuleIOInputs inputs) {
        desiredState = inputs.desiredState;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                steerPosition,
                steerVelocity,
                steerCurrentDraw,
                steerOutput
        );

        Logger.recordOutput(name + "/State", getState());
        Logger.recordOutput(name + "/DesiredState", desiredState);
        Logger.recordOutput(name + "/Position", getPosition());

        Logger.recordOutput(name + "/DriveVelocity", driveVelocity.getValueAsDouble());
        Logger.recordOutput(name + "/DriveVelocityMPS", driveVelocity.getValueAsDouble() * DRIVE_POSITION_COEFFICIENT);
        Logger.recordOutput(name + "/SteerVelocity", steerVelocity.getValueAsDouble());
        Logger.recordOutput(name + "/SteerOutput", steerOutput.getValueAsDouble());
        Logger.recordOutput(name + "/SteerCurrentDraw", steerCurrentDraw.getValueAsDouble());
    }

    @Override
    public void updateDesiredState() {
        double steerAngle = getSteerAngle();

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(steerAngle));

        desiredState.angle = Rotation2d.fromRotations((desiredState.angle.getRotations() % 1.0 + 1.0) % 1.0);

        driveMotor.setControl(driveMotorVelocity.withVelocity(desiredState.speedMetersPerSecond / DRIVE_POSITION_COEFFICIENT));
        steerMotor.setControl(steerMotorPosition.withPosition(desiredState.angle.getRotations()));
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }
}
