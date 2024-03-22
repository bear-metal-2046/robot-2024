package org.tahomarobotics.robot.auto;

import java.util.HashMap;

public class AutoConstants {
    public static final String DEFAULT_AUTO_NAME = "No Operation";

    public static final HashMap<String, double[]> SHOT_TABLE = new HashMap<>() {{
        put("WAVY-6S-CBA-12", new double[]{
                47.00390625,
                47.187101013269495,
                43.98862815970471,
                31.8994556346769,
                33.322191916602785,
                33.354378414301713,
                33.304378414301713
        });
        put("4S-345", new double[]{
                33.59848922325562,
                32.41771429458064,
                33.02018923012028,
                32.092946579612635
        });
    }};
}
