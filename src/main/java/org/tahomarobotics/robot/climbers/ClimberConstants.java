package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    public static final double ZERO_VOLTAGE = -0.75;
    public static final double POSITION_COEFFICIENT = (12.0 / 60.0) * (18.0 / 74.0) * (2 * Math.PI * 0.0254) * 1.25; // Rotations to Meters

    public static final double POSITION_EPSILON = 0.001;
    public static final double VELOCITY_EPSILON = 0.001;

    public static final double CLIMB_MAX_VEL = 0.25; // meter per sec
    public static final double CLIMB_MAX_ACCEL = 1;
    public static final double CLIMB_MAX_JERK = 5;

    public static final double TOP_POSITION = 0.5;

    public static final double BOTTOM_POSITION = 0.0;

    public static final int LADEN_SLOT = 0;
    public static final int UNLADEN_SLOT = 1;

    public static final TalonFXConfiguration CLIMB_CONFIGURATION = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs() // Laden slot
                    .withKP(0)
                    .withKD(0)
                    .withKI(0))
            .withSlot1(new Slot1Configs() // Unladen slot
                    .withKP(0)
                    .withKD(0)
                    .withKI(0))
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
