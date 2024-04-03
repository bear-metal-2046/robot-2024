package org.tahomarobotics.robot.auto;

import java.util.HashMap;

public class AutoConstants {
    public static final String DEFAULT_AUTO_NAME = "No Operation";

    public static final HashMap<String, double[]> SHOT_TABLE = new HashMap<>() {{
        put("WAVY-6S-CBA-12", new double[]{
                38.97,
                40.0,
                38.56,
                32.0,
                23.4,
                23.4,
                23.4
        });
        put("5S-C-345", new double[]{
                38.97,
                33.0,
                25.728,
                25.728,
                25.728
        });
        put("4S-345", new double[]{
                28.665,
                25.728,
                25.728,
                25.728
        });
        put("4S-432", new double[]{
                28.665,
                25.728,
                25.728,
                25.728
        });
        put("4S-453", new double[]{
                28.665,
                25.728,
                25.728,
                25.728
        });
        put("4S-534", new double[]{
                28.665,
                25.728,
                25.728,
                25.728
        });
    }};
}
