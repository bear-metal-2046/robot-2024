package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    void logOutputs();

    default void zero() {}

    Rotation2d getYaw();
}
