package org.tahomarobotics.robot.amp;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.identity.RobotIdentity;

public class AmpArmConstants {
    static final double POSITION_TOLERANCE = 0.1;

    // ARM

    static final double ARM_GEAR_REDUCTION = (16.0 / 64.0) * (18.0 / 72.0) * (16.0 / 48.0);

    static final double ARM_MIN_POSE = -90d/360d;
    static final double ARM_MAX_POSE = 90d/360d;

    public static final double ARM_STOW_POSE = -0.25; // Rotations
    public static final double ARM_AMP_POSE = 0.437255859375; // Rotations
    public static final double ARM_TRAP_POSE = 0.3; // Rotations
    public static final double ARM_SOURCE_POSE = 0.396484375; // Rotations

    static final TalonFXConfiguration armMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs() {{
                GravityType = GravityTypeValue.Arm_Cosine;
            }}
                    .withKP(59.808)
                    .withKD(9.8046)
                    .withKS(0.072852)
                    .withKV(5.3303)
                    .withKA(0.73507)
                    .withKG(0.1853))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicAcceleration(10)
                    .withMotionMagicJerk(50))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / ARM_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    // WRIST

    static final double WRIST_GEAR_REDUCTION = switch (RobotIdentity.getInstance().getRobotID()) {
        case PLAYBEAR_CARTI -> (12.0 / 60.0);
        default -> (12.0 / 72.0);
    };

    public static final double WRIST_MOVING_POSE = 0.18798828125; // Rotations
    public static final double WRIST_STOW_POSE = 0; // Rotations
    public static final double WRIST_AMP_POSE = 0.366455078125; // Rotations
    public static final double WRIST_TRAP_POSE = 0.75; // Rotations
    public static final double WRIST_SOURCE_POSE = 0.158203125; // Rotations

    static final TalonFXConfiguration wristMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(51.509)
                    .withKD(1)
                    .withKS(-0.14318)
                    .withKV(0.73388)
                    .withKA(0.12356)
            )
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(5)
                    .withMotionMagicAcceleration(20)
                    .withMotionMagicJerk(75))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / WRIST_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    // ROLLERS

    static final double ROLLER_GEAR_REDUCTION = switch (RobotIdentity.getInstance().getRobotID()) {
        case PLAYBEAR_CARTI -> (12.0 / 60.0);
        default -> (12.0 / 72.0);
    };

    static final double VELOCITY_TOLERANCE = 0.75;

    static final TalonFXConfiguration rollerMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0.93798)
                    .withKS(0.054983)
                    .withKV(0.6432)
                    .withKA(0.27624))
            .withSlot1(new Slot1Configs()
                    .withKP(56.797)
                    .withKD(5.1735)
                    .withKS(0.054983)
                    .withKV(0.6432)
                    .withKA(0.27624))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(CollectorConstants.COLLECT_MAX_RPS)
                    .withMotionMagicAcceleration(CollectorConstants.COLLECT_MAX_ACCEL)
                    .withMotionMagicJerk(CollectorConstants.COLLECT_MAX_JERK))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / ROLLER_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
