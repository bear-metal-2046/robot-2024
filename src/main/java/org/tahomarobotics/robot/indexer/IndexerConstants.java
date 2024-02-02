package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.shooter.ShooterConstants;

public class IndexerConstants {
    static final double COLLECT_SPEED = CollectorConstants.COLLECT_MAX_RPS - 20; // Rotations
    static final double INTAKE_DISTANCE = 2.0; // Rotations
    static final double TRANSFER_DISTANCE = 5.0; // Rotations
    static final double POSITION_TOLERANCE = 0.1; // Rotations
    static final double FACTOR = 1.75; // For indexing into shooter faster


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
                    .withMotionMagicCruiseVelocity(CollectorConstants.COLLECT_MAX_RPS * FACTOR)
                    .withMotionMagicAcceleration(CollectorConstants.COLLECT_MAX_ACCEL * FACTOR)
                    .withMotionMagicJerk(CollectorConstants.COLLECT_MAX_JERK * FACTOR))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
