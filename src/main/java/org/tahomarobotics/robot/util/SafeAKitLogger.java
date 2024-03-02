package org.tahomarobotics.robot.util;

import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import java.nio.BufferOverflowException;
import java.util.Arrays;

public class SafeAKitLogger {
    public static <T extends WPISerializable> void recordOutput(String key, T value) {
        try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    public static void recordOutput(String key, String value) {
        try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    @SafeVarargs
    public static <T extends StructSerializable> void recordOutput(String key, T... value) {
        try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + Arrays.toString(value) + ")", false);
        }
    }

    public static void recordOutput(String key, double value) {
        try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    public static void recordOutput(String key, boolean value) {
        try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

    public static <E extends Enum<E>> void recordOutput(String key, E value) {
        try {
            Logger.recordOutput(key, value);
        } catch (BufferOverflowException e) {
            DriverStation.reportError("Received a BufferOverflow exception when recording: (" + key + ", " + value + ")", false);
        }
    }

}
