package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;

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

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);

    private final RobustConfigurator configurator;

    @AutoLog
    static class SwerveModuleIOInputs {
        public SwerveModuleState desiredState = new SwerveModuleState();
    }

    // CONSTRUCTOR

    SwerveModuleIO(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        logger = LoggerFactory.getLogger("SwerveModule." + descriptor.moduleName());
        name = "Chassis/Modules/" + descriptor.moduleName();
        configurator = new RobustConfigurator(logger);

        this.angularOffset = angularOffset;

        driveMotor = new TalonFX(descriptor.driveId(), RobotConfiguration.CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), RobotConfiguration.CANBUS_NAME);
        steerAbsEncoder = new CANcoder(descriptor.encoderId(), RobotConfiguration.CANBUS_NAME);

        configurator.configureTalonFX(driveMotor, driveMotorConfiguration);
        configurator.configureTalonFX(steerMotor, steerMotorConfiguration, descriptor.encoderId());
        configurator.configureCancoder(steerAbsEncoder, encoderConfiguration, angularOffset);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = steerAbsEncoder.getAbsolutePosition();
        steerVelocity = steerAbsEncoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY,
                drivePosition, driveVelocity, steerPosition,
                steerVelocity
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerAbsEncoder);
    }

    // CALIBRATION

    
    void initializeCalibration() {
        configurator.setCancoderAngularOffset( steerAbsEncoder, 0);
        configurator.setMotorNeutralMode(steerMotor, NeutralModeValue.Coast);
    }

    
    double finalizeCalibration() {
        angularOffset = -steerPosition.refresh().getValue();
        configurator.setCancoderAngularOffset(steerAbsEncoder, angularOffset);
        configurator.setMotorNeutralMode(steerMotor, NeutralModeValue.Brake);
        return angularOffset;
    }

    
    void cancelCalibration() {
        configurator.setCancoderAngularOffset(steerAbsEncoder, angularOffset);
    }

    // Getters

    /**
     * @return Position of the module in <strong>meters</strong>.
     */
    double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(drivePosition.refresh(), driveVelocity.refresh()) * DRIVE_POSITION_COEFFICIENT;
    }

    /**
     * @return Velocity of the module in <strong>meters/s</strong>.
     */
    double getDriveVelocity() {
        return driveVelocity.refresh().getValue() * DRIVE_POSITION_COEFFICIENT;
    }

    /**
     * @return Relative rotation of the module in <strong>rotations</strong>.
     */
    double getSteerAngle() {
        return BaseStatusSignal.getLatencyCompensatedValue(steerPosition.refresh(), steerVelocity.refresh());
    }

    /**
     * @return The current state of the module.
     */
    
    SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(getSteerAngle()));
    }

    
    SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * @return The current position of the module.
     */
    
    SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(getSteerAngle()));
    }

    // State

    
    void processInputs(SwerveModuleIOInputs inputs) {
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

    
    void updateDesiredState() {
        double steerAngle = getSteerAngle();

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(steerAngle));

        desiredState.angle = Rotation2d.fromRotations((desiredState.angle.getRotations() % 1.0 + 1.0) % 1.0);

        driveMotor.setControl(driveMotorVelocity.withVelocity(desiredState.speedMetersPerSecond / DRIVE_POSITION_COEFFICIENT));
        steerMotor.setControl(steerMotorPosition.withPosition(desiredState.angle.getRotations()));
    }

    
    void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    
    List<BaseStatusSignal> getStatusSignals() {
        return List.of(
                drivePosition,
                driveVelocity,
                steerPosition,
                steerVelocity
        );
    }
}
