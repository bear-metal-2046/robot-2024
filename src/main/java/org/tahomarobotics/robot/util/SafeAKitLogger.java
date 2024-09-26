package org.tahomarobotics.robot.util;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import org.littletonrobotics.junction.Logger;

public class SafeAKitLogger {
    public static <T extends WPISerializable> void recordOutput(String key, T value) {
        try {
            Logger.recordOutput(key, value);
        } catch (Exception ignored) {
        }
    }

    public static void recordOutput(String key, String value) {
        try {
            Logger.recordOutput(key, value);
        } catch (Exception ignored) {
        }
    }

    public static <T extends StructSerializable> void recordOutput(String key, T[] value) {
        try {
            Logger.recordOutput(key, value);
        } catch (Exception ignored) {
        }
    }

    public static void recordOutput(String key, double value) {
        try {
            Logger.recordOutput(key, value);
        } catch (Exception ignored) {
        }
    }

    public static void recordOutput(String key, boolean value) {
        try {
            Logger.recordOutput(key, value);
        } catch (Exception ignored) {
        }
    }

    public static <E extends Enum<E>> void recordOutput(String key, E value) {
        try {
            Logger.recordOutput(key, value);
        } catch (Exception ignored) {
        }
    }

}
