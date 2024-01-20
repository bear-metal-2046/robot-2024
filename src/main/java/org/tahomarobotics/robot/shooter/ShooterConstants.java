package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static org.tahomarobotics.robot.chassis.ChassisConstants.kV_DRIVE;

public class ShooterConstants {
    public static final double KP_SHOOTER = 0.15;
    public static final double KI_SHOOTER = 0.0;
    public static final double KD_SHOOTER = 0.0;

    public static final double KP_INDEX = 0.15;
    public static final double KI_INDEX = 0.0;
    public static final double KD_INDEX = 0.0;

    private static final TalonFXConfiguration shooterMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(KP_SHOOTER)
                    .withKI(KI_SHOOTER)
                    .withKD(KD_SHOOTER))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    public static void configureShootMotor(TalonFXConfigurator configurator) {
        configurator.apply(shooterMotorConfiguration);
    }

    private static final TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(KP_INDEX)
                    .withKI(KI_INDEX)
                    .withKD(KD_INDEX))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    public static void configureIndexMotor(TalonFXConfigurator configurator) {
        configurator.apply(indexMotorConfiguration);
    }
}
