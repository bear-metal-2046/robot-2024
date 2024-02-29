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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLog;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.ToggledOutputs;

import java.util.List;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

public class SwerveModuleIO implements ToggledOutputs {
    private static final Logger logger = LoggerFactory.getLogger(SwerveModuleIO.class);

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

    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> steerCurrent;

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0).withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0).withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO);

    private final RobustConfigurator configurator;

    @AutoLog
    static class SwerveModuleIOInputs {
        public SwerveModuleState desiredState = new SwerveModuleState();
    }

    // CONSTRUCTOR

    SwerveModuleIO(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        name = "Chassis/Modules/" + descriptor.moduleName();
        configurator = new RobustConfigurator(logger);

        this.angularOffset = angularOffset;

        driveMotor = new TalonFX(descriptor.driveId(), RobotConfiguration.CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), RobotConfiguration.CANBUS_NAME);
        steerAbsEncoder = new CANcoder(descriptor.encoderId(), RobotConfiguration.CANBUS_NAME);

        configurator.configureTalonFX(driveMotor, driveMotorConfiguration, descriptor.moduleName() + " drive motor");
        configurator.configureTalonFX(steerMotor, steerMotorConfiguration, descriptor.encoderId(), descriptor.moduleName() + " steer motor");
        configurator.configureCancoder(steerAbsEncoder, encoderConfiguration, angularOffset, descriptor.moduleName() + " cancoder");

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = steerAbsEncoder.getAbsolutePosition();
        steerVelocity = steerAbsEncoder.getVelocity();
        driveCurrent = driveMotor.getSupplyCurrent();
        steerCurrent = steerMotor.getSupplyCurrent();

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
        return BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity) * DRIVE_POSITION_COEFFICIENT;
    }

    /**
     * @return Velocity of the module in <strong>meters/s</strong>.
     */
    double getDriveVelocity() {
        return driveVelocity.getValue() * DRIVE_POSITION_COEFFICIENT;
    }

    /**
     * @return Relative rotation of the module in <strong>rotations</strong>.
     */
    double getSteerAngle() {
        return BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);
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
        recordOutput(name + "/State", getState());
        recordOutput(name + "/DesiredState", desiredState);
        recordOutput(name + "/Position", getPosition());

        recordOutput(name + "/DriveVelocity", driveVelocity.getValueAsDouble());
        recordOutput(name + "/DriveVelocityMPS", driveVelocity.getValueAsDouble() * DRIVE_POSITION_COEFFICIENT);
        recordOutput(name + "/SteerVelocity", steerVelocity.getValueAsDouble());
    }

    
    void updateDesiredState() {
        double steerAngle = getSteerAngle();

        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(steerAngle));

        desiredState.angle = Rotation2d.fromRotations((desiredState.angle.getRotations() % 1.0 + 1.0) % 1.0);
        desiredState.speedMetersPerSecond *= desiredState.angle.minus(Rotation2d.fromRotations(steerAngle)).getCos();

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

    @Override
    public boolean logOutputs() {
        return SmartDashboard.getBoolean("Outputs/SwerveModules", OutputsConfiguration.SWERVE_MODULE);
    }

    public double getTotalCurrent() {
        return driveCurrent.getValue() + steerCurrent.getValue();
    }
}
