package org.tahomarobotics.robot.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class WristConstants {
    static final double WRIST_GEAR_REDUCTION = (12.0 / 72.0) * (18.0 / 24.0) * (24.0 / 18.0);
    static final double STOW_POSE = 0; // Rotations
    static final double TRANS_POSE = 0.1; // Rotations
    static final double AMP_POSE = 0.5; // Rotations
    static final double TRAP_POSE = 0.75; // Rotations

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
                    .withMotionMagicCruiseVelocity(2)
                    .withMotionMagicAcceleration(10)
                    .withMotionMagicJerk(50))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / WRIST_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
