package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ClimberConstants {
    public static final double ZERO_SPEED = -1;
    public static final double POSITION_COEFFICIENT = 0;

    public static final double POSITION_EPSILON = 0.05;

    public static final double TOP_POSITION = 0.0;

    public static final double BOTTOM_POSITION = 0.0;

    public static final TalonFXConfiguration CLIMB_CONFIGURATION = new TalonFXConfiguration();
    public static final Slot0Configs CLIMB_UNLADEN_SLOT = new Slot0Configs()
            .withKP(0).withKD(0).withKI(0);
    public static final Slot0Configs CLIMB_LADEN_SLOT = new Slot0Configs()
            .withKP(0).withKD(0).withKI(0);
}
