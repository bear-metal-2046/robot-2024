package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.util.RobustConfigurator;

class Climber{

    private final Logger logger;
    private final TalonFX climbMotor;

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    public Climber(int motorID, String name, boolean isInverted) {
        logger = LoggerFactory.getLogger(name);

        RobustConfigurator configurator = new RobustConfigurator(logger);
        climbMotor = new TalonFX(motorID);
        configurator.configureTalonFX(climbMotor, ClimberConstants.CLIMB_CONFIGURATION
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast)
                ));
    }

    //MOTOR CONTROLERY

    public void setTargetPos(double targetPosition, int slot) {
        climbMotor.setControl(positionControl.withPosition(targetPosition).withSlot(slot));
    }

    public void runWithVoltage(double targetVoltage) {
        climbMotor.setControl(new VoltageOut(targetVoltage));
    }

    public void zeroAtCurrentPosition() {
        climbMotor.setPosition(0);
    }

    public void stop() {
        climbMotor.stopMotor();
    }


    // GETTERS

    public TalonFX getMotor() {
        return climbMotor;
    }

    public double getPosition() {
        return climbMotor.getPosition().refresh().getValue();
    }

    public double getVelocity() {
        return climbMotor.getVelocity().getValueAsDouble();
    }
}

