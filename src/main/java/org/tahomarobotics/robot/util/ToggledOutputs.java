package org.tahomarobotics.robot.util;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unchecked")
public interface ToggledOutputs {
    boolean logOutputs();

    default <T extends WPISerializable> void recordOutput(String key, T value) {
        if (logOutputs())
            Logger.recordOutput(key, value);
    }

    default <T extends StructSerializable> void recordOutput(String key, T... value) {
        if (logOutputs())
            Logger.recordOutput(key, value);
    }

    default void recordOutput(String key, double value) {
        if (logOutputs())
            Logger.recordOutput(key, value);
    }

    default void recordOutput(String key, boolean value) {
        if (logOutputs())
            Logger.recordOutput(key, value);
    }

    default <E extends Enum<E>> void recordOutput(String key, E value) {
        if (logOutputs())
            Logger.recordOutput(key, value);
    }
}
