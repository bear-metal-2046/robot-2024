package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.tahomarobotics.robot.identity.RobotIdentity;

import java.util.function.Supplier;

public class ShooterConstants {
    public static final double SHOOTER_SPEED; // rps
    public static final double IDLE_SPEED; // rps
    public static final double TRANSFER_VELOCITY = 10; // rps
    public static final double REVERSE_INTAKE_VELOCITY = 40; // rps

    public static final double TIME_SHOT_OFFSET_POSITIVE = 0.4;  // The number of seconds forward in time the shooter
                                                                 // should calculate for when moving away from the speaker
    public static final double TIME_SHOT_OFFSET_NEGATIVE = 0.5; // The same as ^ but for moving towards the speaker

    static final double SHOOTER_SPEED_TOLERANCE = 5; // rps
    public static final double MAX_PIVOT_ANGLE;
    public static final double MIN_PIVOT_ANGLE = Units.degreesToRotations(4);

    public static final double CLOSE_REDUNDANT_ANGLE = 0.139;
    public static final double FAR_REDUNDANT_ANGLE = 0.0916;

    static final double PIVOT_ANGLE_TOLERANCE = Units.degreesToRotations(2.5);
    public static final double SHOOTER_COLLECT_PIVOT_ANGLE = MIN_PIVOT_ANGLE;

    static final double PIVOT_GEAR_REDUCTION;
    static final double GEAR_REDUCTION_COMPENSATION;
    static final double SHOOTER_GEAR_REDUCTION;

    static final InvertedValue PIVOT_INVERSION;

    static {
        switch (RobotIdentity.robotID) {
            case PLAYBEAR_CARTI, BEARITONE -> {
                SHOOTER_SPEED = 5000.0 / 60.0;
                IDLE_SPEED = SHOOTER_SPEED;
                SHOOTER_GEAR_REDUCTION = (24.0 / 18.0);
                PIVOT_INVERSION = InvertedValue.Clockwise_Positive;
            }
            default -> {
                SHOOTER_SPEED = 75.0;
                IDLE_SPEED = 37.5;
                SHOOTER_GEAR_REDUCTION = 1.0;
                PIVOT_INVERSION = InvertedValue.CounterClockwise_Positive;
            }
        }

        switch (RobotIdentity.robotID) {
            case BEARITONE -> {
                MAX_PIVOT_ANGLE = Units.degreesToRotations(56.338);
                PIVOT_GEAR_REDUCTION = (12.0 / 44.0) * (30.0 / 36.0) * (10.0 / 80.0);
                GEAR_REDUCTION_COMPENSATION = 1.02272727301;
            }
            default -> {
                MAX_PIVOT_ANGLE = Units.degreesToRotations(52.4);
                PIVOT_GEAR_REDUCTION = (10.0 / 44.0) * (30.0 / 36.0) * (10.0 / 80.0);
                GEAR_REDUCTION_COMPENSATION = 1;
            }
        }
    }

    public static final double LOW_PASS_POS = MIN_PIVOT_ANGLE;
    public static final double HIGH_PASS_SPEED = 55;
    public static final double HIGH_PASS_POS = Units.degreesToRotations(40);

//    public static final double SHOT_SPEED = Units.inchesToMeters(Units.rotationsToRadians(SHOOTER_SPEED) * (1.75)) * 0.8; // meters/sec
//    public static final double PASS_SPEED = Units.inchesToMeters(Units.rotationsToRadians(HIGH_PASS_SPEED) * (1.75)) * 0.8; // meters/sec

    static final double BIAS_AMT = Units.degreesToRotations(.5);

    static final double STATOR_CURRENT_LIMIT_AUTO = 80.0;
    static final double SUPPLY_CURRENT_LIMIT_AUTO = 60.0;

    static final double STATOR_CURRENT_LIMIT_TELEOP = 45.0;
    static final double SUPPLY_CURRENT_LIMIT_TELEOP = 30.0;

    public static final Translation2d SHOOTER_PIVOT_OFFSET = new Translation2d(0.1238, 0.1899);

    private static final Translation2d RED_SPEAKER_TARGET_POSITION = new Translation2d(16.53, 5.55);
    private static final Translation2d BLUE_SPEAKER_TARGET_POSITION = new Translation2d(0, 5.55);
    public static final Supplier<Translation2d> SPEAKER_TARGET_POSITION = () ->
            DriverStation
                    .getAlliance()
                    .filter(a -> a == DriverStation.Alliance.Blue)
                    .map(a -> BLUE_SPEAKER_TARGET_POSITION).orElse(RED_SPEAKER_TARGET_POSITION);

    private static final Translation2d RED_PASS_TARGET_POSITION = new Translation2d(16.541, 8.211);
    private static final Translation2d BLUE_PASS_TARGET_POSITION = new Translation2d(0, 8.211);
    public static final Supplier<Translation2d> PASS_TARGET_POSITION = () ->
            DriverStation
                    .getAlliance()
                    .filter(a -> a == DriverStation.Alliance.Blue)
                    .map(a -> BLUE_PASS_TARGET_POSITION).orElse(RED_PASS_TARGET_POSITION);

    static final TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AUTO)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AUTO)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot0Configs()
                        .withKP(0.007663)
//                        .withKI(0.0)
                        .withKS(0.015371)
                        .withKV(0.090404)
                        .withKA(0.015002);
                default -> new Slot0Configs()
                        .withKP(0.086027)
                        .withKS(0.077906)
                        .withKV(0.11218)
                        .withKA(0.012355);
            })
//            .withClosedLoopRamps(new ClosedLoopRampsConfigs()
//                    .withVoltageClosedLoopRampPeriod(1.0))                            // This should make the current after shot significantly less
            .withMotorOutput(new MotorOutputConfigs()                                  // I'm afraid it will affect autos tho
                    .withNeutralMode(NeutralModeValue.Brake)                           // Might just want to only do it during teleop
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / SHOOTER_GEAR_REDUCTION))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(120.0)
                    .withMotionMagicJerk(500.0))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    static final TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs() {{
                        GravityType = GravityTypeValue.Arm_Cosine;
                    }}
                    .withKP(400.0)
                    .withKI(20.0)
                    .withKD(4.312)
                    .withKS(0.1677)
                    .withKV(2.1175)
                    .withKA(/*1.496*/0.0)
                    .withKG(0.32768))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(PIVOT_INVERSION))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1.5)
                    .withMotionMagicAcceleration(5)
                    .withMotionMagicJerk(100))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / PIVOT_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
