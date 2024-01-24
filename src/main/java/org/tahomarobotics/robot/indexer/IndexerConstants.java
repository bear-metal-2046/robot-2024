package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
    static final double INTAKE_DISTANCE = 2.0; // Rotations
    static final double TRANSFER_DISTANCE = 1.0; // Rotations

    static final double IDLE_VEL = 25.0; // rps

    static final TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(0.0054238)
                    .withKS(0.15023)
                    .withKV(0.11194)
                    .withKA(0.0022556))
            .withSlot1(new Slot1Configs()
                    .withKP(11.676)
                    .withKD(0.16184)
                    .withKS(0.15023)
                    .withKV(0.11194)
                    .withKA(0.0022556))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(25)
                    .withMotionMagicAcceleration(50)
                    .withMotionMagicJerk(100))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
