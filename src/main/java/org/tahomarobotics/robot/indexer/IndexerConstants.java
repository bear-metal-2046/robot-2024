package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.collector.CollectorConstants;

public class IndexerConstants {
    static final double INTAKE_DISTANCE = 2.0; // Rotations
    static final double TRANSFER_DISTANCE = 5.0; // Rotations
    static final double POSITION_TOLERANCE = 0.1; // Rotations


    // Supply current is current that’s being drawn at the input bus voltage
    // Useful for preventing breakers from tripping in the PDP
    private final static double SUPPLY_CURRENT_LIMIT = 40;

    // Stator current is current that’s being drawn by the motor
    // Useful for limiting rotor acceleration/heat production
    private final static double STATOR_CURRENT_LIMIT = 60;

    static final TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
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
                    .withMotionMagicCruiseVelocity(CollectorConstants.COLLECT_MAX_RPS)
                    .withMotionMagicAcceleration(CollectorConstants.COLLECT_MAX_ACCEL)
                    .withMotionMagicJerk(CollectorConstants.COLLECT_MAX_JERK))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
