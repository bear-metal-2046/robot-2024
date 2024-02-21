package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public enum ATCamera {
        COLLECTOR_LEFT("Collector Left", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)
                ),
                new Rotation3d(
                        0, 0, Units.degreesToRadians(15)
                )
        )),
        COLLECTOR_RIGHT("Collector Right", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)
                ),
                new Rotation3d(
                        0, 0, Units.degreesToRadians(-15)
                )
        )),
        SHOOTER_LEFT("Shooter Left", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(3), Units.inchesToMeters(3.7145), Units.inchesToMeters(27.5)
                ),
                new Rotation3d(
                        0, Units.degreesToRadians(-25), Units.degreesToRadians(162)
                )
        )),
        SHOOTER_RIGHT("Shooter Right", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(3), Units.inchesToMeters(-3.7145), Units.inchesToMeters(27.5)
                ),
                new Rotation3d(
                        0, Units.degreesToRadians(-25), Units.degreesToRadians(198)
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
