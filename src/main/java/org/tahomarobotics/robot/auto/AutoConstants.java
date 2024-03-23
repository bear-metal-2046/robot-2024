package org.tahomarobotics.robot.auto;

import java.util.HashMap;

public class AutoConstants {
    public static final String DEFAULT_AUTO_NAME = "No Operation";

    public static final HashMap<String, double[]> SHOT_TABLE = new HashMap<>() {{
        put("WAVY-6S-CBA-12", new double[]{
                47.00,
                47.18,
                43.98,
                40.82,
                33.92,
                33.95,
                33.95
        });
        put("4S-345", new double[]{
                33.59848922325562,
                32.41771429458064,
                33.02018923012028,
                32.092946579612635
        });
    }};
}
