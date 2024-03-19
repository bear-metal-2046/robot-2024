package org.tahomarobotics.robot.auto;

import java.util.HashMap;

public class AutoConstants {
    public static final String DEFAULT_AUTO_NAME = "No Operation";

    public static final HashMap<String, double[]> SHOT_TABLE = new HashMap<>() {{
        put("WAVY-6S-CBA-12", new double[]{
                51.50390625,
                48.487101013269495,
                44.08862815970471,
                32.93994556346769,
                28.922191916602785,
                28.83275849501904,
                29.104378414301713
        });
    }};
}
