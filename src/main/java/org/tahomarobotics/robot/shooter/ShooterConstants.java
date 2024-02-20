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
    public static final double TRANSFER_VELOCITY = 10; // rps

    public static final double TIME_SHOT_OFFSET = 0.5;

    static final double SHOOTER_SPEED_TOLERANCE = 2.5; // rps
    public static final double MAX_PIVOT_ANGLE;
    public static final double MIN_PIVOT_ANGLE = 0.01;
    static final double PIVOT_ANGLE_TOLERANCE = 0.0025;
    public static final double SHOOTER_COLLECT_PIVOT_ANGLE = MIN_PIVOT_ANGLE;

    static final double PIVOT_GEAR_REDUCTION = (14.0 / 56.0) * (10.0 / 90.0);
    static final double SHOOTER_GEAR_REDUCTION;

    static final InvertedValue PIVOT_INVERSION;

    static {
        switch (RobotIdentity.getInstance().getRobotID()) {
            case PLAYBEAR_CARTI, BEARITONE -> {
                SHOOTER_SPEED = 100.0;
                MAX_PIVOT_ANGLE = Units.degreesToRotations(51.50390625);
                SHOOTER_GEAR_REDUCTION = (30.0 / 18.0);
                PIVOT_INVERSION = InvertedValue.Clockwise_Positive;
            }
            default -> {
                SHOOTER_SPEED = 75.0;
                MAX_PIVOT_ANGLE = Units.degreesToRotations(50.4);
                SHOOTER_GEAR_REDUCTION = 1.0;
                PIVOT_INVERSION = InvertedValue.CounterClockwise_Positive;
            }
        }
    }

    public static final double SHOT_SPEED = Units.inchesToMeters(Units.rotationsToRadians(SHOOTER_SPEED) * (1.75)) * 0.5; // meters/sec

    static final double BIAS_AMT = Units.degreesToRotations(5) / 50;

    static final double STATOR_CURRENT_LIMIT = 80.0;
    static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final Translation2d SHOOTER_PIVOT_OFFSET = new Translation2d(0.1238, 0.1899);

    private static final Translation2d RED_SPEAKER_TARGET_POSITION = new Translation2d(16.53, 5.55);
    private static final Translation2d BLUE_SPEAKER_TARGET_POSITION = new Translation2d(0, 5.55);
    public static final Supplier<Translation2d> SPEAKER_TARGET_POSITION = () ->
            DriverStation
                    .getAlliance()
                    .filter(a -> a == DriverStation.Alliance.Blue)
                    .map(a -> BLUE_SPEAKER_TARGET_POSITION).orElse(RED_SPEAKER_TARGET_POSITION);

    static final TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(switch (RobotIdentity.getInstance().getRobotID()) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot0Configs()
                        .withKP(.076223)
                        .withKS(.10456)
                        .withKV(.071642)
                        .withKA(.015732);
                default -> new Slot0Configs()
                        .withKP(0.086027)
                        .withKS(0.077906)
                        .withKV(0.11218)
                        .withKA(0.012355);
            })
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / SHOOTER_GEAR_REDUCTION))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(160)
                    .withMotionMagicJerk(1000))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    static final TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs() {{
                        GravityType = GravityTypeValue.Arm_Cosine;
                    }}
                    .withKP(52.816)
                    .withKD(5.4426)
                    .withKS(0.041681)
                    .withKV(3.8296)
                    .withKA(0.26584)
                    .withKG(0.32742))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(PIVOT_INVERSION))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicAcceleration(10)
                    .withMotionMagicJerk(50))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / PIVOT_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
