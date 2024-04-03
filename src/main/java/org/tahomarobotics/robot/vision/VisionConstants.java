package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public enum Camera {
        COLLECTOR_LEFT("Collector Left", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(7.15), Units.inchesToMeters(3.28), 0.65
                ),
                new Rotation3d(
                        0, Units.degreesToRadians(20), 0
                )
        )),
        COLLECTOR_RIGHT("Collector Right", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(7.15), Units.inchesToMeters(-3.28), 0.65
                ),
                new Rotation3d(
                        0, Units.degreesToRadians(-15), 0
                )
        )),
        SHOOTER_LEFT("Shooter Left", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(3), Units.inchesToMeters(3.7145), Units.inchesToMeters(26.125)
                ),
                new Rotation3d(
                        0, Units.degreesToRadians(-25), Units.degreesToRadians(162)
                )
        )),
        SHOOTER_RIGHT("Shooter Right", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(3), Units.inchesToMeters(-3.7145), Units.inchesToMeters(26.125)
                ),
                new Rotation3d(
                        0, Units.degreesToRadians(-25), Units.degreesToRadians(198)
                )
        ));

        public final Transform3d offset;
        public final String cameraName;

        Camera(String cameraName, Transform3d offset) {
            this.offset = offset;
            this.cameraName = cameraName;
        }
    }

    public static final double POSE_DIFFERENCE_THRESHOLD = 2;
    public static final double DEGREES_DIFFERENCE_THRESHOLD = 60;
    public static final double TARGET_DISTANCE_THRESHOLD = 8;
    public static final double SINGLE_TARGET_DISTANCE_THRESHOLD = 4;
    public final static double MAX_VALID_REPROJECTION_ERROR = 1.0;
    public static final double MAX_VALID_AMBIGUITY = 0.2;
    public final static double MAX_HEIGHT_METERS = 0.1;
    public final static double MAX_ROLL_PITCH = Units.degreesToRadians(10);
    public final static double FIELD_LENGTH = 16.541;
    public final static double FIELD_WIDTH = 8.211;
    public final static double MIN_SNAPSHOT_DELAY = 5.0;
}