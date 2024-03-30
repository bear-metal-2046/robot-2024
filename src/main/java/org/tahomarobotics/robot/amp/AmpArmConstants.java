package org.tahomarobotics.robot.amp;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.identity.RobotID;
import org.tahomarobotics.robot.identity.RobotIdentity;

public class AmpArmConstants {
    static final double POSITION_TOLERANCE = 0.01;

    public static final double SOURCE_COLLECT_CURRENT = 5;
    public static final double SOURCE_COLLECT_ACCEL_TIME = 0.25;

    // ARM

    static final double ARM_GEAR_REDUCTION;

    static final double ARM_MIN_POSE = -90d / 360d;
    static final double ARM_MAX_POSE = 155d / 360d;

    public static final double ARM_STOW_POSE, ARM_AMP_POSE, ARM_TRAP_POSE,
            ARM_SOURCE_POSE, ARM_CLIMB_POSE;

    static {
        ARM_CLIMB_POSE = Units.degreesToRotations(150);
        ARM_GEAR_REDUCTION = RobotIdentity.robotID == RobotID.BEARITONE ?
                (14.0 / 64.0) * (18.0 / 72.0) * (16.0 / 48.0) :
                (16.0 / 64.0) * (18.0 / 72.0) * (16.0 / 48.0);

        switch (RobotIdentity.robotID) {
            case PLAYBEAR_CARTI, BEARITONE -> {
                ARM_STOW_POSE = Units.degreesToRotations(-90.00);
                ARM_AMP_POSE = Units.degreesToRotations(72.51);
                ARM_TRAP_POSE = Units.degreesToRotations(62.66);
                ARM_SOURCE_POSE = Units.degreesToRotations(67.939453125);
            }
            default -> {
                ARM_STOW_POSE = Units.degreesToRotations(-90.00);
                ARM_AMP_POSE = Units.degreesToRotations(157.41);
                ARM_TRAP_POSE = Units.degreesToRotations(108.00);
                ARM_SOURCE_POSE = Units.degreesToRotations(142.734375);
            }
        }
    }

    // WRIST

    static final double WRIST_GEAR_REDUCTION;
    static final InvertedValue WRIST_INVERT;

    public static final double WRIST_MOVING_POSE, WRIST_AMP_POSE, WRIST_TRAP_POSE, WRIST_SOURCE_POSE;
    public static final double WRIST_STOW_POSE = 0;

    // The angle at which the wrist exits moving pose and transitions to the following state.
    public static final double WRIST_MOVING_POSE_THRESHOLD = Units.degreesToRotations(2);

    static {
        switch (RobotIdentity.robotID) {
            case PLAYBEAR_CARTI, BEARITONE -> {
                WRIST_MOVING_POSE = Units.degreesToRotations(123.22265625);
                WRIST_AMP_POSE = Units.degreesToRotations(145.7421875 + 3.0);
                WRIST_TRAP_POSE = Units.degreesToRotations(139.314453125 + 10.0);
                WRIST_SOURCE_POSE = Units.degreesToRotations(76.2890625);
            }
            default -> {
                WRIST_MOVING_POSE = Units.degreesToRotations(67.67578125);
                WRIST_AMP_POSE = Units.degreesToRotations(131.923828125);
                WRIST_TRAP_POSE = Units.degreesToRotations(270);
                WRIST_SOURCE_POSE = Units.degreesToRotations(56.953125);
            }
        }

        WRIST_INVERT = RobotIdentity.robotID == RobotID.PLAYBEAR_CARTI ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    }

    static {
        switch (RobotIdentity.robotID) {
            case BEARITONE, PLAYBEAR_CARTI -> WRIST_GEAR_REDUCTION = (12d / 60d) * (24d / 18d);
            default -> WRIST_GEAR_REDUCTION = 12d / 72d;
        }
    }

    // ROLLER

    static final double ROLLER_GEAR_REDUCTION;
    static final InvertedValue ROLLER_INVERT;

    public static final double NOTE_INTAKE_POSITION = 1;
    public static final double SOURCE_INTAKE_DISTANCE = 2;

    public static final double TRAP_VELOCITY = 4.5;
    public static final double REVERSE_INTAKE_VELOCITY = 10;

    static {
        switch (RobotIdentity.robotID) {
            case BEARITONE -> {
                ROLLER_GEAR_REDUCTION = 12d / 60d;
                ROLLER_INVERT = InvertedValue.CounterClockwise_Positive;
            }
            case PLAYBEAR_CARTI -> {
                ROLLER_GEAR_REDUCTION = 12d / 60d;
                ROLLER_INVERT = InvertedValue.Clockwise_Positive;
            }
            default -> {
                ROLLER_GEAR_REDUCTION = 12d / 72d;
                ROLLER_INVERT = InvertedValue.CounterClockwise_Positive;
            }
        }
    }

    // CONFIGURATIONS

    // ARM

    static final TalonFXConfiguration armMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot0Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKP(58.611)
                        .withKD(8.7205)
                        .withKS(0.17479)
                        .withKV(4.8674)
                        .withKA(0.59266)
                        .withKG(0.15206);
                default -> new Slot0Configs()
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withKP(59.808)
                        .withKD(9.8046)
                        .withKS(0.072852)
                        .withKV(5.3303)
                        .withKA(0.73507)
                        .withKG(0.1853);
            })
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1)
                    .withMotionMagicAcceleration(10)
                    .withMotionMagicJerk(50))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / ARM_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    // WRIST

    static final TalonFXConfiguration wristMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(51.509)
                    .withKD(1)
                    .withKS(-0.14318)
                    .withKV(0.73388)
                    .withKA(0.12356))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(WRIST_INVERT))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(5)
                    .withMotionMagicAcceleration(20)
                    .withMotionMagicJerk(75))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / WRIST_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    // ROLLER

    static final TalonFXConfiguration rollerMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot0Configs()
                        .withKP(0.3706)
                        .withKS(0.066923)
                        .withKV(0.57611)
                        .withKA(0.21145);
                default -> new Slot0Configs()
                        .withKP(0.93798)
                        .withKS(0.054983)
                        .withKV(0.6432)
                        .withKA(0.27624);
            })
            .withSlot1(switch (RobotIdentity.robotID) {
                case BEARITONE, PLAYBEAR_CARTI -> new Slot1Configs()
                        .withKP(55.146)
                        .withKD(1)
                        .withKS(0.066923)
                        .withKV(0.57611)
                        .withKA(0.21145);
                default -> new Slot1Configs()
                        .withKP(56.797)
                        .withKD(5.1735)
                        .withKS(0.054983)
                        .withKV(0.6432)
                        .withKA(0.27624);
            })
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(ROLLER_INVERT))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(CollectorConstants.COLLECT_MAX_RPS)
                    .withMotionMagicAcceleration(CollectorConstants.COLLECT_MAX_ACCEL)
                    .withMotionMagicJerk(CollectorConstants.COLLECT_MAX_JERK))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / ROLLER_GEAR_REDUCTION))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
