package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.identity.RobotIdentity;

public class CollectorConstants {

    private final static double DEPLOY_GEAR_REDUCTION = (10d / 72d) * (16d / 40d);
    public final static double COLLECT_GEAR_REDUCTION = (18d / 36d);

    //Deploy Motion Profile Constraints
    private final static double DEPLOY_MAX_RPS = 2.5;
    private final static double DEPLOY_MAX_ACCEL = DEPLOY_MAX_RPS / 0.25;
    private final static double DEPLOY_MAX_JERK = DEPLOY_MAX_ACCEL / 0.125;

    //Collection Motion Profile Constraints
    public final static double COLLECT_MAX_RPS = 40;
    public final static double COLLECT_MAX_ACCEL = COLLECT_MAX_RPS / 0.25;
    public final static double COLLECT_MAX_JERK = COLLECT_MAX_ACCEL / 0.125;

    public final static double STOW_POSITION, COLLECT_POSITION, EJECT_POSITION, ZERO_POSITION;

    static {
        switch (RobotIdentity.robotID) {
            case PLAYBEAR_CARTI, BEARITONE -> {
                STOW_POSITION = Units.degreesToRotations(90.0000000);
                COLLECT_POSITION = Units.degreesToRotations(211.46484375);
                EJECT_POSITION = Units.degreesToRotations(183.69140625);
                ZERO_POSITION = Units.degreesToRotations(73.037109375);
            }
            default -> {
                // TODO: Get these positions.
                STOW_POSITION = Units.degreesToRotations(10);
                COLLECT_POSITION = Units.degreesToRotations(137.5);
                EJECT_POSITION = Units.degreesToRotations(100);
                ZERO_POSITION = Units.degreesToRotations(80);
            }
        }
    }
    
    private final static double SUPPLY_CURRENT_LIMIT = 40;
    private final static double STATOR_CURRENT_LIMIT = 80;

    static final TalonFXConfiguration collectMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot0Configs()
                        .withKP(.11795)
                        .withKS(.27235)
                        .withKV(.2485)
                        .withKA(.010979);
                default -> new Slot0Configs()
                        .withKP(0.23711)
                        .withKS(0.56717)
                        .withKV(0.23279)
                        .withKA(0.025363);
            })
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
            .withSlot0(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot0Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKP(55.903)
                        .withKD(4.8502)
                        .withKS(0.086158)
                        .withKV(1.8207)
                        .withKA(0.25676)
                        .withKG(0.25583);
                default -> new Slot0Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKP(53.655)
                        .withKD(4.0302)
                        .withKS(0.098451)
                        .withKV(1.7039)
                        .withKA(0.18671)
                        .withKG(0.2808);
            })
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
