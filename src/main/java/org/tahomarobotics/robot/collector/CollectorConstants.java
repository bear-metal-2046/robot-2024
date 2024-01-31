package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class CollectorConstants {

    private final static double DEPLOY_GEAR_REDUCTION = (10d / 72d) * (16d / 40d);
    public final static double COLLECT_GEAR_REDUCTION = (18d / 36);

    //Deploy Motion Profile Constraints
    private final static double DEPLOY_MAX_RPS = 2.5;
    private final static double DEPLOY_MAX_ACCEL = DEPLOY_MAX_RPS / 0.25;
    private final static double DEPLOY_MAX_JERK = DEPLOY_MAX_ACCEL / 0.125;

    //Collection Motion Profile Constraints
    public final static double COLLECT_MAX_RPS = 40;
    public final static double COLLECT_MAX_ACCEL = COLLECT_MAX_RPS / 0.25;
    public final static double COLLECT_MAX_JERK = COLLECT_MAX_ACCEL / 0.125;

    public final static double STOW_POSITION = Units.degreesToRotations(10);
    public final static double COLLECT_POSITION = Units.degreesToRotations(137.5);
    public final static double EJECT_POSITION = Units.degreesToRotations(100);

    public final static double EPSILON = 0.01;

    // Supply current is current that’s being drawn at the input bus voltage
    // Useful for preventing breakers from tripping in the PDP
    private final static double SUPPLY_CURRENT_LIMIT = 40;

    // Stator current is current that’s being drawn by the motor
    // Useful for limiting rotor acceleration/heat production
    private final static double STATOR_CURRENT_LIMIT = 60;

    static final TalonFXConfiguration collectMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withKP(0.23711)
                    .withKS(0.56717)
                    .withKV(0.23279)
                    .withKA(0.025363))
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
            .withSlot0(new Slot0Configs() {{
                        GravityType = GravityTypeValue.Arm_Cosine;
                    }}
                    .withKP(53.655)
                    .withKD(4.0302)
                    .withKS(0.098451)
                    .withKV(1.7039)
                    .withKA(0.18671)
                    .withKG(0.2808)
            )
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
