package org.tahomarobotics.robot.util;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import java.nio.BufferOverflowException;

@SuppressWarnings("unchecked")
public interface ToggledOutputs {
    boolean logOutputs();

    default <T extends WPISerializable> void recordOutput(String key, T value) {
        if (logOutputs()) try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    default <T extends StructSerializable> void recordOutput(String key, T... value) {
        if (logOutputs()) try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    default void recordOutput(String key, double value) {
        if (logOutputs()) try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    default void recordOutput(String key, boolean value) {
        if (logOutputs()) try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    default <E extends Enum<E>> void recordOutput(String key, E value) {
        if (logOutputs()) try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }
}
