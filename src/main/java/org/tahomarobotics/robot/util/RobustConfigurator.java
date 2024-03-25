package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.function.Supplier;

public class RobustConfigurator {

    private static final int RETRIES = 5;
    private final Logger logger;

    private String detail;

    public static StatusCode retryConfigurator(Supplier<StatusCode> func, String succeed, String fail, String retry) {
        final Logger logger = LoggerFactory.getLogger(RobustConfigurator.class);

        return retryConfigurator(logger, func, succeed, fail, retry);
    }

    public static StatusCode retryConfigurator(Logger logger, Supplier<StatusCode> func, String succeed, String fail, String retry) {
        boolean success = false;
        StatusCode statusCode = null;
        for (int i = 0; i < RETRIES; i++) {
            statusCode = func.get();
            if (statusCode == StatusCode.OK) {
                success = true;
                break;
            }
            logger.warn(retry);
        }
        if (success) {
            logger.info(succeed);
        } else {
            logger.error(fail);
        }

        return statusCode;
    }

    private void retryMotorConfigurator(Supplier<StatusCode> func) {
        retryConfigurator(logger, func,
                "Successful motor configuration" + detail,
                "Failed motor configuration" + detail,
                "Retrying failed motor configuration" + detail
        );
    }

    public RobustConfigurator(Logger logger) {
        this.logger = logger;
        this.detail = "";
    }

    public RobustConfigurator(Logger logger, String name) {
        this.logger = logger;
        this.detail = " for " + name;
    }

    public void configureTalonFX(TalonFX motor, TalonFXConfiguration configuration) {
        var configurator = motor.getConfigurator();
        retryMotorConfigurator(() -> configurator.apply(configuration));
    }

    public void configureTalonFX(TalonFX motor, TalonFXConfiguration configuration, String device) {
        var configurator = motor.getConfigurator();
        this.detail = " for " + device;
        retryMotorConfigurator(() -> configurator.apply(configuration));
    }

    public void configureTalonFX(TalonFX motor, TalonFXConfiguration configuration, int encoderId) {
        configuration.Feedback.FeedbackRemoteSensorID = encoderId;
        configureTalonFX(motor, configuration);
    }

    public void configureTalonFX(TalonFX motor, TalonFXConfiguration configuration, int encoderId, String device) {
        configuration.Feedback.FeedbackRemoteSensorID = encoderId;
        this.detail = " for " + device;
        configureTalonFX(motor, configuration);
    }

    public void configureTalonFX(TalonFX motor, TalonFXConfiguration configuration, TalonFX motorFollower, boolean isOppositeMasterDirection) {
        configureTalonFX(motor, configuration);
        configureTalonFX(motorFollower, configuration);

        motorFollower.setControl(new Follower(motor.getDeviceID(), isOppositeMasterDirection));
    }

    public void setMotorNeutralMode(TalonFX motor, NeutralModeValue mode) {
        var configurator = motor.getConfigurator();
        var configuration = new MotorOutputConfigs();
        retryMotorConfigurator(() -> configurator.refresh(configuration));

        configuration.withNeutralMode(mode);
        retryMotorConfigurator(() -> configurator.apply(configuration));

    }

    public void configureCancoder(CANcoder encoder, MagnetSensorConfigs configuration, double angularOffset) {
        configuration.withMagnetOffset(angularOffset);
        var configurator = encoder.getConfigurator();
        retryMotorConfigurator(() -> configurator.apply(configuration));
    }

    public void configureCancoder(CANcoder encoder, MagnetSensorConfigs configuration, double angularOffset, String device) {
        configuration.withMagnetOffset(angularOffset);
        this.detail = " for " + device;
        var configurator = encoder.getConfigurator();
        retryMotorConfigurator(() -> configurator.apply(configuration));
    }

    public void setCancoderAngularOffset(CANcoder encoder, double angularOffset) {
        var configurator = encoder.getConfigurator();
        var configuration = new MagnetSensorConfigs();
        retryMotorConfigurator(() -> configurator.refresh(configuration));

        configuration.withMagnetOffset(angularOffset);
        retryMotorConfigurator(() -> configurator.apply(configuration));
    }
}