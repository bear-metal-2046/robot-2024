package org.tahomarobotics.robot;

public class RobotConfiguration {
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final boolean IS_REPLAY = false;
    public static final String CANBUS_NAME = "CANivore";
    public static final boolean USING_PHOENIX_PRO = true;
    public static final double ODOMETRY_UPDATE_FREQUENCY = 250;

    public static Mode getMode() {
        return Robot.isReal() ? Mode.REAL : (IS_REPLAY ? Mode.REPLAY : Mode.SIM);
    }
}
