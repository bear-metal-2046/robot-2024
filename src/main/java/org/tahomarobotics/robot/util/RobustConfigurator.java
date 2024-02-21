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

import java.util.function.Supplier;

public class RobustConfigurator {

    private static final int RETRIES = 5;
    private final Logger logger;

    private String detail;

    private void retryConfigurator(Supplier<StatusCode> func) {
        boolean success = false;
        for(int i = 0; i < RETRIES; i++) {
            StatusCode statusCode = func.get();
            if (statusCode == StatusCode.OK) {
                success = true;
                break;
            }
            logger.warn("Retrying failed motor configuration" + detail);
        }
        if (success) {
            logger.info("Successful motor configuration" + detail);
        } else {
            logger.error("Failed motor configuration" + detail);
        }
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
        retryConfigurator(() -> configurator.apply(configuration));
    }

    public void configureTalonFX(TalonFX motor, TalonFXConfiguration configuration, String device) {
        var configurator = motor.getConfigurator();
        this.detail = " for " + device;
        retryConfigurator(() -> configurator.apply(configuration));
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
        retryConfigurator(() -> configurator.refresh(configuration));

        configuration.withNeutralMode(mode);
        retryConfigurator(() -> configurator.apply(configuration));

    }

    public void configureCancoder(CANcoder encoder, MagnetSensorConfigs configuration, double angularOffset) {
        configuration.withMagnetOffset(angularOffset);
        var configurator = encoder.getConfigurator();
        retryConfigurator(() -> configurator.apply(configuration));
    }

    public void configureCancoder(CANcoder encoder, MagnetSensorConfigs configuration, double angularOffset, String device) {
        configuration.withMagnetOffset(angularOffset);
        this.detail = " for " + device;
        var configurator = encoder.getConfigurator();
        retryConfigurator(() -> configurator.apply(configuration));
    }

    public void setCancoderAngularOffset(CANcoder encoder, double angularOffset) {
        var configurator = encoder.getConfigurator();
        var configuration = new MagnetSensorConfigs();
        retryConfigurator(() -> configurator.refresh(configuration));

        configuration.withMagnetOffset(angularOffset);
        retryConfigurator(() -> configurator.apply(configuration));
    }
}