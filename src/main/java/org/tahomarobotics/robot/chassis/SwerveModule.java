package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

public class SwerveModule {
    // Member Variables

    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final String name;
    private final Translation2d translationOffset;

    // Constructor

    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        io = switch (RobotConfiguration.getMode()) {
            case REAL -> new SwerveModuleIOReal(descriptor, angularOffset);
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

    public Translation2d getTranslationOffset() {
        return translationOffset;
    }

    // State

    public void setDesiredState(SwerveModuleState desiredState) {
        inputs.desiredState = desiredState;

        io.processInputs(inputs);
        io.updateDesiredState();
    }

    public void periodic() {
        io.periodic();

        io.processInputs(inputs);
        Logger.processInputs("Chassis/Modules/" + name, inputs);
    }

    public void stop() {
        io.stop();
    }
}
