package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    void processInputs(SwerveModuleIOInputs inputs);

    void updateDesiredState();

    SwerveModulePosition getPosition();

    SwerveModuleState getState();

    default void periodic() {}

    default void initializeCalibration() {}
    default double finalizeCalibration() { return 0.0; }
    default void cancelCalibration() {}

    default void stop() {}

    @AutoLog
    class SwerveModuleIOInputs {
        public SwerveModuleState desiredState = new SwerveModuleState();
    }
}
