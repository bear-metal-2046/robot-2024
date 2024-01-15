package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

import java.util.Collections;
import java.util.List;

public interface SwerveModuleIO {
    void processInputs(SwerveModuleIOInputs inputs);

    void updateDesiredState();

    SwerveModulePosition getPosition();

    SwerveModuleState getState();
    SwerveModuleState getDesiredState();

    default void periodic() {}

    default void initializeCalibration() {}
    default double finalizeCalibration() { return 0.0; }
    default void cancelCalibration() {}

    default void stop() {}

    default List<BaseStatusSignal> getStatusSignals() {
        return Collections.emptyList();
    }

    @AutoLog
    class SwerveModuleIOInputs {
        public SwerveModuleState desiredState = new SwerveModuleState();
    }
}
