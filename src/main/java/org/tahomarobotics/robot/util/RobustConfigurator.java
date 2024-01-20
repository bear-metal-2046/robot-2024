package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.slf4j.Logger;

import java.util.function.Supplier;

import static org.tahomarobotics.robot.chassis.ChassisConstants.encoderConfiguration;

public class RobustConfigurator {

    private static final int RETRIES = 5;

    private static void retryConfigurator(Logger logger, String name, Supplier<StatusCode> func) {
        boolean success = false;
        for(int i = 0; i < RETRIES; i++) {
            StatusCode statusCode = func.get();
            if (statusCode == StatusCode.OK) {
                success = true;
                break;
            }
            logger.warn("Retrying failed motor configuration for " + name);
        }
        if (success) {
            logger.info("Successful motor configuration for " + name);
        } else {
            logger.error("Failed motor configuration for " + name);
        }
    }

    public static void configureTalonFX(Logger logger, String name, TalonFX motor, TalonFXConfiguration configuration) {
        var configurator = motor.getConfigurator();
        retryConfigurator(logger, name, () -> configurator.apply(configuration));
    }

    public static void configureTalonFX(Logger logger, String name, TalonFX motor, TalonFXConfiguration configuration, int encoderId) {
        configuration.Feedback.FeedbackRemoteSensorID = encoderId;
        configureTalonFX(logger, name, motor, configuration);
    }

    public static void setMotorNeutralMode(Logger logger, String name, TalonFX motor, NeutralModeValue mode) {
        var configurator = motor.getConfigurator();
        var configuration = new MotorOutputConfigs();
        retryConfigurator(logger, name, () -> configurator.refresh(configuration));

        configuration.withNeutralMode(mode);
        retryConfigurator(logger, name, () -> configurator.apply(configuration));

    }

    public static void configureCancoder(Logger logger, String name, CANcoder encoder, MagnetSensorConfigs configuration, double angularOffset) {
        configuration.withMagnetOffset(angularOffset);
        var configurator = encoder.getConfigurator();
        retryConfigurator(logger, name, () -> configurator.apply(configuration));
    }

    public static void setCancoderAngularOffset(Logger logger, String name, CANcoder encoder, double angularOffset) {
        var configurator = encoder.getConfigurator();
        var configuration = new MagnetSensorConfigs();
        retryConfigurator(logger, name, () -> configurator.refresh(configuration));

        configuration.withMagnetOffset(angularOffset);
        retryConfigurator(logger, name, () -> configurator.apply(configuration));
    }
}
