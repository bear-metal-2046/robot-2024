package org.tahomarobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

public class RobotMap {
    public static final int PIGEON = 0;

    // SWERVE MODULES on CANCODER
    public final static SwerveModuleDescriptor FRONT_LEFT_MOD = new SwerveModuleDescriptor(
            "Front Left", FRONT_LEFT_OFFSET, 1, 11, 21);
    public final static SwerveModuleDescriptor FRONT_RIGHT_MOD = new SwerveModuleDescriptor(
            "Front Right", FRONT_RIGHT_OFFSET, 2, 12, 22);
    public final static SwerveModuleDescriptor BACK_LEFT_MOD = new SwerveModuleDescriptor(
            "Back Left", BACK_LEFT_OFFSET, 3, 13, 23);
    public final static SwerveModuleDescriptor BACK_RIGHT_MOD = new SwerveModuleDescriptor(
            "Back Right", BACK_RIGHT_OFFSET, 4, 14, 24);

    public final static int BEAM_BREAK_ONE = 1;

    public final static int BEAM_BREAK_TWO = 2;

    public static final int LEFT_SHOOTER_MOTOR = 31;
    public static final int RIGHT_SHOOTER_MOTOR = 32;
    public static final int SHOOTER_PIVOT_MOTOR = 33;

    public static final int INDEXER_MOTOR = 34;

    public final static int DEPLOY_MOTOR_LEFT = 35;
    public final static int DEPLOY_MOTOR_RIGHT = 36;
    public final static int COLLECTOR_MOTOR = 37;
    public final static int LEFT_CLIMB_MOTOR = 41;
    public final static int RIGHT_CLIMB_MOTOR = 42;

    public final static int ARM_MOTOR = 38;
    public final static int WRIST_MOTOR = 39;
    public final static int ROLLERS_MOTOR = 40;

    public record SwerveModuleDescriptor(String moduleName, Translation2d offset, int driveId, int steerId,
                                         int encoderId) {
    }
}
