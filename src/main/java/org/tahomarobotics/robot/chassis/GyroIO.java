package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Collections;
import java.util.List;

public interface GyroIO {
    default void zero() {}

    Rotation2d getYaw();

    default List<BaseStatusSignal> getStatusSignals() {
        return Collections.emptyList();
    }
}
