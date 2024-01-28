package org.tahomarobotics.robot.rollers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.collector.CollectorConstants;

public class RollerConstants {
    static final double ROLLER_GEAR_REDUCTION = (12.0 / 72.0) * (18.0 / 24.0) * (24.0 / 18.0);
    static final double ROLLER_INTAKE_DISTANCE = 2.0; // Rotations
    static final double ROLLER_TRANSFER_DISTANCE = 5.0; // Rotations
    static final double ROLLER_POSITION_TOLERANCE = 0.1; // Rotations

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
