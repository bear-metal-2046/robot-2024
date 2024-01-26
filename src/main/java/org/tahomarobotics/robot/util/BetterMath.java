package org.tahomarobotics.robot.util;

public class BetterMath {
    public static final double TAU = Math.PI / 2;

    public static double clamp(double n, double min, double max) {
        return Math.min(max, Math.max(n, min));
    }
}
