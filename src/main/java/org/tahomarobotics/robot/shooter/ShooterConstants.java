package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class ShooterConstants {
    static final double SHOOTER_SPEED = 100; // rps
    static final double SHOOTER_SPEED_TOLERANCE = 0.25; // rps
    static final double MAX_PIVOT_ANGLE = 0.14;

    static final double PIVOT_GEAR_RATIO = (14.0 / 56.0) * (56.0 / 10.0) * (10.0 / 90.0);

    private static final double SHOOTER_HEIGHT = 1.0;
    private static final double SPEAKER_HEIGHT = 4.0;
    static final double SPEAKER_HEIGHT_DIFF = SPEAKER_HEIGHT - SHOOTER_HEIGHT;

    private static final Translation2d RED_SPEAKER_TARGET_POSITION = new Translation2d();
    private static final Translation2d BLUE_SPEAKER_TARGET_POSITION = new Translation2d();
    static final Supplier<Translation2d> SPEAKER_TARGET_POSITION = () ->
            DriverStation
                    .getAlliance()
                    .filter(a -> a != DriverStation.Alliance.Blue)
                    .map(a -> RED_SPEAKER_TARGET_POSITION).orElse(BLUE_SPEAKER_TARGET_POSITION);

    static final TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0.15)
                    .withKI(0.0)
                    .withKD(0.0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(10)
                    .withMotionMagicJerk(50))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    static final TalonFXConfiguration pivotMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0.15)
                    .withKI(0.0)
                    .withKD(0.0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.1)
                    .withMotionMagicAcceleration(1)
                    .withMotionMagicJerk(5))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
