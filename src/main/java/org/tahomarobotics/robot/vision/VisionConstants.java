package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    private static final double BAR_HEIGHT_IN = 26.0;

    public enum ATCamera {
        LEFT("Left", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0), Units.inchesToMeters(BAR_HEIGHT_IN), Units.inchesToMeters(0)
                ),
                new Rotation3d(
                        0, 0, Units.degreesToRadians(15)
                )
        )),
        RIGHT("Right", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0), Units.inchesToMeters(BAR_HEIGHT_IN), Units.inchesToMeters(0)
                ),
                new Rotation3d(
                        0, 0, Units.degreesToRadians(-15)
                )
        )),
        BACK("Back", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(2), Units.inchesToMeters(0), Units.inchesToMeters(BAR_HEIGHT_IN)
                        ),
                new Rotation3d(
                        0, 0, Units.degreesToRadians(180)
                )
        ));

        public final Transform3d offset;
        public final String cameraName;

        ATCamera(String cameraName, Transform3d offset) {
            this.offset = offset;
            this.cameraName = cameraName;
        }
    }

    public static final double POSE_DIFFERENCE_THRESHOLD = 2;
    public static final double DEGREES_DIFFERENCE_THRESHOLD = 60;
    public static final double TARGET_DISTANCE_THRESHOLD = 8;
    public static final double SINGLE_TARGET_DISTANCE_THRESHOLD = 4;
}