package org.tahomarobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

public class RobotMap {
    public static final int PIGEON = 0;

    // SWERVE MODULES on CANCODER
    public final static SwerveModuleDescriptor FRONT_LEFT_MOD = new SwerveModuleDescriptor(
            "Front-Left", FRONT_LEFT_OFFSET, 1, 11, 21);
    public final static SwerveModuleDescriptor FRONT_RIGHT_MOD = new SwerveModuleDescriptor(
            "Front-Right", FRONT_RIGHT_OFFSET, 2, 12, 22);
    public final static SwerveModuleDescriptor BACK_LEFT_MOD = new SwerveModuleDescriptor(
            "Back-Left", BACK_LEFT_OFFSET, 3, 13, 23);
    public final static SwerveModuleDescriptor BACK_RIGHT_MOD = new SwerveModuleDescriptor(
            "Back-Right", BACK_RIGHT_OFFSET, 4, 14, 24);

    public final static int BEAM_BREAK = 0;

    public static final int TOP_SHOOTER_MOTOR = 31;
    public static final int BOTTOM_SHOOTER_MOTOR = 32;
    public static final int SHOOTER_PIVOT_MOTOR = 33;

    public static final int INDEXER_MOTOR = 34;

    public record SwerveModuleDescriptor(String moduleName, Translation2d offset, int driveId, int steerId,
                                         int encoderId) {
    }
}
