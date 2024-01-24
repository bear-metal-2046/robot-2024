package org.tahomarobotics.robot.util;

public class BetterMath {
    public static double clamp(double n, double min, double max) {
        return Math.min(max, Math.max(n, min));
    }
}
