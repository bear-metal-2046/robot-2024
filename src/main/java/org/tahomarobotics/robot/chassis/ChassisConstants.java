/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * <p>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * <p>
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 * <p>
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.identity.RobotIdentity;

import java.util.stream.IntStream;

/**
 * Constants for the chassis.
 * Uses default SI units with angles in rotations.
 */
@SuppressWarnings({"SuspiciousNameCombination", "HungarianNotationConstants"})
public final class ChassisConstants {
    public static final double TRACK_WIDTH = 0.52705;
    public static final double WHEELBASE = 0.52705;
    public static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    public static final double HALF_WHEELBASE = WHEELBASE / 2;

    public static final double DRIVE_RADIUS = Math.sqrt((HALF_TRACK_WIDTH * HALF_TRACK_WIDTH) + (HALF_WHEELBASE * HALF_WHEELBASE));

    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);

    public static final double WHEEL_RADIUS;

    public static final double DRIVE_REDUCTION;
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    static {
        switch (RobotIdentity.robotID) {
            case BEARITONE -> {
                DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
                WHEEL_RADIUS  = 0.04912216;
            }
            case PLAYBEAR_CARTI -> {
                DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
                WHEEL_RADIUS  = 0.05024;
            }
            default -> {
                DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
                WHEEL_RADIUS = 0.04;
            }
        }
    }

    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    public static final double DRIVE_POSITION_COEFFICIENT = WHEEL_CIRCUMFERENCE * DRIVE_REDUCTION; // r/s -> m/s

    //Placeholder PID values
    public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(2.5,0,0.5);
    public static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(5, 0, 0.5);

    private static final double SHOOT_ROTATION_TARGET_TOLERANCE = Units.degreesToRadians(5.0);

    public static final PIDController SHOOT_MODE_CONTROLLER = new PIDController(12.5, 0, 0.5) {{
        setTolerance(SHOOT_ROTATION_TARGET_TOLERANCE);
    }};

    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 55.0; // Amps
    public static final double DRIVE_STATOR_CURRENT_LIMIT = 120.0;
    public static final double STEER_SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double STEER_STATOR_CURRENT_LIMIT = 80.0;

    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
    public static final double MAX_VELOCITY = SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION * WHEEL_RADIUS * 1.1; // m/s
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE); // r/s

    public static final double TRANSLATION_LIMIT = 6.0;
    public static final double ROTATION_LIMIT = TRANSLATION_LIMIT / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);

    public static final PathConstraints AUTO_PATHFINDING_CONSTRAINTS = new PathConstraints(
            2.0,
            2.0,
            540,
            720);


    public static final double MASS = 25.33313;

    // volts per mps
    public static final double kV_DRIVE = (2 * Math.PI) / SWERVE_DRIVE_MOTOR.KvRadPerSecPerVolt;

    // volts per mps^2
    // ohms * meter * kg / Nm * Amps = volts * kg / N = volts * kg / (kg m/s^2) =
    // volts / m/s2
    @SuppressWarnings("unused")
    public static final double kA_DRIVE = SWERVE_DRIVE_MOTOR.rOhms * WHEEL_RADIUS * MASS
            / (DRIVE_REDUCTION * SWERVE_DRIVE_MOTOR.KtNMPerAmp);

    /// DEVICE CONFIGURATION

    public static final MagnetSensorConfigs encoderConfiguration = new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    public static final TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(DRIVE_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withKP(0.15)
                    .withKV(kV_DRIVE))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    public static final TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(STEER_SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withKP(8.0)
                    .withKI(0.01)
                    .withKD(0.16)
            )
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{
                ContinuousWrap = true;
            }})
            .withFeedback(new FeedbackConfigs() {{
                              if (RobotConfiguration.CANIVORE_PHOENIX_PRO) {
                                  FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                                  RotorToSensorRatio = 1 / STEER_REDUCTION;
                              } else {
                                  FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                              }
                          }}
            );

    public static Pose2d[] BLUE_STAGE_CHAIN_POSES = new Pose2d[] {
            new Pose2d(new Translation2d(4.2347, 3.0079), Rotation2d.fromDegrees(-120)), // Source Side
            new Pose2d(new Translation2d(4.2347, 5.2033), Rotation2d.fromDegrees(120)),  // Amp Side
            new Pose2d(new Translation2d(6.1360, 4.1056), Rotation2d.fromDegrees(0)) // Far Side
    };

    public static Pose2d[] RED_STAGE_CHAIN_POSES = new Pose2d[] {
            new Pose2d(new Translation2d(12.3063, 3.0079), Rotation2d.fromDegrees(-60)), // Source Side
            new Pose2d(new Translation2d(12.3063, 5.2033), Rotation2d.fromDegrees(60)), // Amp Side
            new Pose2d(new Translation2d(10.4050, 4.1056), Rotation2d.fromDegrees(180))  // Far Side
    };

    public static PathConstraints CLIMB_MOVEMENT_CONSTRAINTS = new PathConstraints(
            2.0,
            2.0,
            540,
            720);

    public static Pose2d getClosestChainPose() {
        Pose2d[] poses = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue ?
                BLUE_STAGE_CHAIN_POSES : RED_STAGE_CHAIN_POSES;
        Pose2d currentPos = Chassis.getInstance().getPose();
        Translation2d currentTranslation = currentPos.getTranslation();

        int minIndex = IntStream.range(0, 3).reduce((i, j) -> currentTranslation.getDistance(poses[j].getTranslation()) < currentTranslation.getDistance(poses[i].getTranslation()) ? j : i).getAsInt();

        return poses[minIndex];
    }

    public static double clampAccel(double value) {
        return MathUtil.clamp(value, -TRANSLATION_LIMIT, TRANSLATION_LIMIT);
    }
}
