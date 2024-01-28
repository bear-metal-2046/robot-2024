package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmConstants {
    static final double ARM_GEAR_REDUCTION = (16.0 / 64.0) * (18.0 / 72.0) * (16.0 / 48.0);

    static final double ARM_MIN_POSE = -8d/360d;
    static final double ARM_MAX_POSE = 270d/360d;
    static final double STOW_POSE = 0; // Rotations
    static final double TRANS_POSE = -0.01; // Rotations
    static final double AMP_POSE = 0.25; // Rotations
    static final double TRAP_POSE = 0.3; // Rotations



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
                    .withMotionMagicCruiseVelocity(0.5)
                    .withMotionMagicAcceleration(5)
                    .withMotionMagicJerk(25))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / ARM_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
