package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.identity.RobotIdentity;

public class ClimberConstants {
    public static final double CLOSENESS_THRESHOLD = 0.05;

    public static final double ZERO_VOLTAGE = -0.75;
    public static final double POSITION_COEFFICIENT = switch (RobotIdentity.robotID) {
        case PLAYBEAR_CARTI -> (9.0 / 50.0) * (18.0 / 50.0) * (2 * Math.PI * 0.0281);
        case BEARITONE -> (10.0 / 48.0) * (18.0 / 50.0)  * (2 * Math.PI * 0.0281);
        default -> (12.0 / 60.0) * (18.0 / 74.0) * (2 * Math.PI * 0.0254) * 1.25;
    };

    public static final double POSITION_EPSILON = 0.005;
    public static final double VELOCITY_EPSILON = 0.005;

    public static final double CLIMB_MAX_VEL = 0.5; // meter per sec
    public static final double CLIMB_MAX_ACCEL = 2;
    public static final double CLIMB_MAX_JERK = 10;

    public static final double TOP_POSITION = 0.56;

    public static final double BOTTOM_POSITION = 0.0025;
    public static final double ALMOST_BOTTOM_POSITION = 0.0025 + Units.inchesToMeters(1);

    public static final TalonFXConfiguration CLIMB_CONFIGURATION = new TalonFXConfiguration()
            .withSlot0(switch (RobotIdentity.robotID) {
                case PLAYBEAR_CARTI, BEARITONE -> new Slot0Configs() {{
                    GravityType = GravityTypeValue.Elevator_Static;
                }}
                        .withKP(66.377)
                        .withKD(25.008)
                        .withKS(0.024785)
                        .withKV(9.4982)
                        .withKA(4.9239)
                        .withKG(0.03319);
                default -> new Slot0Configs() {{
                    GravityType = GravityTypeValue.Elevator_Static;
                }}
                        .withKP(67.383)
                        .withKD(31.564)
                        .withKS(0.19558)
                        .withKV(8.6397)
                        .withKA(8.5074)
                        .withKG(0.054765);
            })
            .withSlot1(switch (RobotIdentity.robotID) {
                case PLAYBEAR_CARTI, BEARITONE -> new Slot1Configs() // laden slot
                        .withKP(66.715)
                        .withKI(50)
                        .withKD(26.732)
                        .withKS(0.076721)
                        .withKV(9.3706)
                        .withKA(5.7819)
                        .withKG(-0.70776);
                default -> new Slot1Configs() // laden slot
                        .withKP(0)
                        .withKD(0)
                        .withKI(0);
            })
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(CLIMB_MAX_VEL)
                    .withMotionMagicAcceleration(CLIMB_MAX_ACCEL)
                    .withMotionMagicJerk(CLIMB_MAX_JERK))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1 / POSITION_COEFFICIENT))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
