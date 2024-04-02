package org.tahomarobotics.robot.auto;

import java.util.HashMap;

public class AutoConstants {
    public static final String DEFAULT_AUTO_NAME = "No Operation";

    public static final HashMap<String, double[]> SHOT_TABLE = new HashMap<>() {{
        put("WAVY-6S-CBA-12", new double[]{
                37.97,
                36.56,
                34.38,
                28.21,
                22.4,
                22.4,
                22.4
        });
        put("4S-345", new double[]{
                27.665,
                24.428,
                24.428,
                24.428
        });
        put("4S-432", new double[]{
                33.59848922325562,
                33.51771429458064,
                33.52018923012028,
                33.492946579612635
        });
    }};
}
