package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CollectorConstants {

    public final static double DEPLOY_GEAR_REDUCTION = (12d / 72d) * (16d / 40d);
    public final static double COLLECT_GEAR_REDUCTION = (18d / 36);

    //Deploy Motion Profile Constraints
    public final static double DEPLOY_MAX_RPS = 2;
    public final static double DEPLOY_MAX_ACCEL = DEPLOY_MAX_RPS / 5;
    public final static double DEPLOY_MAX_JERK = DEPLOY_MAX_ACCEL / 2;

    //Collection Motion Profile Constraints
    public final static double COLLECT_MAX_RPS = 50;
    public final static double COLLECT_MAX_ACCEL = COLLECT_MAX_RPS / 2;
    public final static double COLLECT_MAX_JERK = COLLECT_MAX_ACCEL / 5;

    public final static double STOW_POSITION = 0.05;
    public final static double COLLECT_POSITION = 0.28;

    public final static double EPSILON = 0.01;

    private final static double SUPPLY_CURRENT_LIMIT = 40;
    private final static double STATOR_CURRENT_LIMIT = 60;

    static final TalonFXConfiguration collectMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withKP(0.15))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(COLLECT_MAX_RPS)
                    .withMotionMagicAcceleration(COLLECT_MAX_ACCEL)
                    .withMotionMagicJerk(COLLECT_MAX_JERK))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / COLLECT_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    static final TalonFXConfiguration deployMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withKP(2))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(DEPLOY_MAX_RPS)
                    .withMotionMagicAcceleration(DEPLOY_MAX_ACCEL)
                    .withMotionMagicJerk(DEPLOY_MAX_JERK))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / DEPLOY_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
