package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import java.util.List;

public class SwerveModule {
    // Member Variables

    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    public final String name;
    private final Translation2d translationOffset;

    // Constructor

    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        io = switch (RobotConfiguration.getMode()) {
            case REAL -> new SwerveModuleIO(descriptor, angularOffset);
            case SIM, REPLAY -> new SwerveModuleIOSim(descriptor, angularOffset);
        };

        name = descriptor.moduleName();
        translationOffset = descriptor.offset();
    }

    // Calibration

    public void initializeCalibration() {
        io.initializeCalibration();
    }

    public double finalizeCalibration() {
        return io.finalizeCalibration();
    }
    public void cancelCalibration() {
        io.cancelCalibration();
    }

    // Getters

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }
    public SwerveModuleState getState() {
        return io.getState();
    }
    public SwerveModuleState getDesiredState() {
        return io.getDesiredState();
    }
    public SwerveModuleState getAccelerationState() { return io.getAccelerationState(); }
    public SwerveModuleState getRawAccelerationState() { return io.getRawAccelerationState(); }
    public double getSteerCurrent() {
        return io.getSteerCurrent();
    }
    public double getDriveCurrent() {
        return io.getDriveCurrent();
    }

    public List<BaseStatusSignal> getStatusSignals() {
        return io.getStatusSignals();
    }

    public Translation2d getTranslationOffset() {
        return translationOffset;
    }

    // State

    /**
     * Updates the desired state.
     * @param desiredState New state for the module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        inputs.desiredState = desiredState;

        io.processInputs(inputs);
        io.updateDesiredState();
    }

    public void testSteer() {
        io.testSteer();
    }
    public void testDrive() {
        io.testDrive();
    }

    /**
     * Updates the inputs and outputs of the SwerveModule periodically.
     */
    public void periodic() {
        io.periodic();

        io.processInputs(inputs);
        Logger.processInputs("Chassis/Modules/" + name, inputs);
    }

    public void stop() {
        io.stop();
    }

    public double getTotalCurent() { return io.getTotalCurrent(); }
}
