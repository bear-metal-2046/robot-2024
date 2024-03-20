package org.tahomarobotics.robot.auto;

import java.util.HashMap;

public class AutoConstants {
    public static final String DEFAULT_AUTO_NAME = "No Operation";

    public static final HashMap<String, double[]> SHOT_TABLE = new HashMap<>() {{
        put("WAVY-6S-CBA-12", new double[]{
                48.00390625,
                48.487101013269495,
                43.98862815970471,
                29.23994556346769,
                29.122191916602785,
                28.63275849501904,
                29.204378414301713
        });
        put("4S-345", new double[]{
                33.09848922325562,
                32.41771429458064,
                32.62018923012028,
                32.092946579612635
        });
    }};
}
