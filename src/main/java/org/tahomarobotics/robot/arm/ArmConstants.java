package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmConstants {
    static final double STOW_POSE = 0; // Rotations
    static final double TRANS_POSE = 0.1; // Rotations
    static final double AMP_POSE = 0.5; // Rotations

    static final double TRAP_POSE = 0.5; // Rotations

    static final TalonFXConfiguration armMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0)
                    .withMotionMagicAcceleration(0)
                    .withMotionMagicJerk(0))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
