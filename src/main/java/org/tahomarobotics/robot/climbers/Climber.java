package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.util.RobustConfigurator;

import static org.tahomarobotics.robot.climbers.ClimberConstants.BOTTOM_POSITION;
import static org.tahomarobotics.robot.climbers.ClimberConstants.TOP_POSITION;

class Climber{

    private final Logger logger;
    private final TalonFX motor;

    public final StatusSignal<Double> voltage, current;

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    public Climber(int motorID, String name, boolean isInverted) {
        logger = LoggerFactory.getLogger(name);

        RobustConfigurator configurator = new RobustConfigurator(logger);
        motor = new TalonFX(motorID);
        configurator.configureTalonFX(motor, ClimberConstants.CLIMB_CONFIGURATION
                .withMotorOutput(ClimberConstants.CLIMB_CONFIGURATION.MotorOutput
                        .withInverted(isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                ));

        voltage = motor.getMotorVoltage();
        current = motor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                voltage, current);

        motor.optimizeBusUtilization();
    }

    //MOTOR CONTROLERY

    public void setTargetPos(double targetPosition, int slot) {
        motor.setControl(positionControl.withPosition(MathUtil.clamp(targetPosition, BOTTOM_POSITION, TOP_POSITION)).withSlot(slot));
    }

    public void runWithVoltage(double targetVoltage) {
        motor.setControl(new VoltageOut(targetVoltage));
    }

    public void zeroAtCurrentPosition() {
        motor.setPosition(0);
    }

    public void stop() {
        motor.stopMotor();
    }


    // GETTERS

    public TalonFX getMotor() {
        return motor;
    }

    public double getPosition() {
        return motor.getPosition().refresh().getValue();
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }
}

