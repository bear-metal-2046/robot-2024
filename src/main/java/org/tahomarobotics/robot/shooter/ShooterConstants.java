package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class ShooterConstants {
    static final double SHOOTER_SPEED = 75; // rps
    static final double SHOOTER_SPEED_TOLERANCE = 0.25; // rps
    static final double MAX_PIVOT_ANGLE = 0.14;
    static final double MIN_PIVOT_ANGLE = 0.01;
    static final double PIVOT_ANGLE_TOLERANCE = 0.0025;
    public static final double SHOOTER_COLLECT_PIVOT_ANGLE = MIN_PIVOT_ANGLE;

    static final double PIVOT_GEAR_REDUCTION = (14.0 / 56.0) * (10.0 / 90.0);

    static final double BIAS_AMT = Units.degreesToRotations(5) / 50;

    private static final double SHOOTER_HEIGHT = Units.inchesToMeters(8.5);
    private static final double SPEAKER_HEIGHT = Units.inchesToMeters(78.5);
    static final double SPEAKER_HEIGHT_DIFF = SPEAKER_HEIGHT - SHOOTER_HEIGHT;

    private static final Translation2d RED_SPEAKER_TARGET_POSITION = new Translation2d(16.53, 5.55);
    private static final Translation2d BLUE_SPEAKER_TARGET_POSITION = new Translation2d(0, 5.55);
    static final Supplier<Translation2d> SPEAKER_TARGET_POSITION = () ->
            DriverStation
                    .getAlliance()
                    .filter(a -> a == DriverStation.Alliance.Blue)
                    .map(a -> BLUE_SPEAKER_TARGET_POSITION).orElse(RED_SPEAKER_TARGET_POSITION);

    static final TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0.086027)
                    .withKS(0.077906)
                    .withKV(0.11218)
                    .withKA(0.012355))
            .withSlot1(new Slot1Configs()
                    .withKP(27.034)
                    .withKD(0.74151)
                    .withKS(0.077906)
                    .withKV(0.11218)
                    .withKA(0.012355))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
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
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicAcceleration(10)
                    .withMotionMagicJerk(50))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / PIVOT_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
