package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.identity.RobotIdentity;

public class IndexerConstants {
    static final double COLLECT_SPEED = CollectorConstants.COLLECT_MAX_RPS - 25; // Rotations
    static final double INDEX_VELOCITY = 8.0; // Rotations
    static final double TRANSFER_DISTANCE = 5.0; // Rotations
    static final double POSITION_TOLERANCE = 0.1; // Rotations
    static final double FACTOR = 2; // For indexing into shooter faster

    static final double INDEXER_GEAR_REDUCTION;
    static final InvertedValue INVERSION;

    static {
        switch (RobotIdentity.robotID) {
            case PLAYBEAR_CARTI, BEARITONE -> {
                INDEXER_GEAR_REDUCTION = 18d / 30d;
                INVERSION = InvertedValue.CounterClockwise_Positive;
            }
            default -> {
                INDEXER_GEAR_REDUCTION = 1d;
                INVERSION = InvertedValue.Clockwise_Positive;
            }
        }
    }

    static final TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot0Configs()
                        .withKP(0.12161)
                        .withKS(0.17898)
                        .withKV(0.19484)
                        .withKA(0.0057781);
                default -> new Slot0Configs()
                        .withKP(0.0054238)
                        .withKS(0.15023)
                        .withKV(0.11194)
                        .withKA(0.0022556);
            })
            .withSlot1(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot1Configs()
                        .withKP(19.812)
                        .withKD(0.36216)
                        .withKS(0.15023)
                        .withKV(0.11194)
                        .withKA(0.0022556);
                default -> new Slot1Configs()
                        .withKP(11.676)
                        .withKD(0.16184)
                        .withKS(0.15023)
                        .withKV(0.11194)
                        .withKA(0.0022556);
            })
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(INVERSION))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(CollectorConstants.COLLECT_MAX_RPS * FACTOR)
                    .withMotionMagicAcceleration(CollectorConstants.COLLECT_MAX_ACCEL * FACTOR)
                    .withMotionMagicJerk(CollectorConstants.COLLECT_MAX_JERK * FACTOR))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / INDEXER_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
