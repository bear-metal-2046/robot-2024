package org.tahomarobotics.robot.auto;

import org.tahomarobotics.robot.identity.RobotIdentity;

import java.util.HashMap;

public class AutoConstants {
    public static final String DEFAULT_AUTO_NAME = "No Operation";


    public static final HashMap<String, double[]> SHOT_TABLE = switch (RobotIdentity.robotID) {
        case PLAYBEAR_CARTI -> new HashMap<>() {
            {
                for (String name : new String[]{"4S-345", "4S-354", "4S-435", "4S-453", "4S-534", "4S-543"}) {
                    put(name, new double[]{
                            27.165,
                            28.165,
                            28.165,
                            28.165
                    });
                }

                for (String name : new String[]{"4S-234", "4S-324", "4S-423", "4S-432"}) {
                    put(name, new double[]{
                            27.665,
                            27.665,
                            27.665,
                            27.665
                    });
                }
                put("WAVY-6S-CBA-12", new double[]{
                        34.97,
                        40.0,
                        38.0 - 2.0,
                        30.0 - 2.0,
                        21.9 - 1.25,
                        21.4 - 1.25,
                        23.4
                });
                put("5S-C-345", new double[]{
                        38.97,
                        33.0,
                        25.728,
                        25.728,
                        25.728
                });
            }
        };
        default -> new HashMap<>() {
            {
                for (String name : new String[]{"4S-345", "4S-354", "4S-435", "4S-453", "4S-534", "4S-543"}) {
                    put(name, new double[]{
                            27.165 - 1.5,
                            28.165,
                            28.165,
                            28.165
                    });
                }

                for (String name : new String[]{"4S-234", "4S-324", "4S-423", "4S-432"}) {
                    put(name, new double[]{
                            27.665,
                            27.665,
                            27.665,
                            27.665
                    });
                }
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
            }
        };
    };
}
