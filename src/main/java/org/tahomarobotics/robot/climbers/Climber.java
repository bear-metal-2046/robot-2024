package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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

class Climber {
    private final TalonFX motor;
    private final String name;

    public final StatusSignal<Double> position, velocity, current;

    private final MotionMagicVoltage unladenPositionControl = new MotionMagicVoltage(0.0).withSlot(0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage ladenPositionControl = new MotionMagicVoltage(0.0).withSlot(1).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    public Climber(int motorID, String name, boolean isInverted) {
        Logger logger = LoggerFactory.getLogger(name);
        this.name = name;

        RobustConfigurator configurator = new RobustConfigurator(logger);
        motor = new TalonFX(motorID);
        configurator.configureTalonFX(motor, ClimberConstants.CLIMB_CONFIGURATION
                .withMotorOutput(ClimberConstants.CLIMB_CONFIGURATION.MotorOutput
                        .withInverted(isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                ));

        position = motor.getPosition();
        velocity = motor.getVelocity();
        current = motor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                position, velocity, current);

        motor.optimizeBusUtilization();
    }

    // SETTERS

    public void setPositionLaden(double position) {
        motor.setControl(ladenPositionControl.withPosition(MathUtil.clamp(position, BOTTOM_POSITION, TOP_POSITION)));
    }

    public void setPositionUnladen(double position) {
        motor.setControl(unladenPositionControl.withPosition(MathUtil.clamp(position, BOTTOM_POSITION, TOP_POSITION)));
    }

    public void setVoltage(double targetVoltage) {
        motor.setControl(new VoltageOut(targetVoltage));
    }

    public void zero() {
        RobustConfigurator.retryConfigurator(() -> motor.setPosition(0),
                "Zeroed " + name + " Motor",
                "FAILED TO SET " + name + " POSITION",
                "Retrying setting " + name + " position.");
    }

    public void stop() {
        motor.stopMotor();
    }

    // GETTERS

    public double getPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(position.refresh(), velocity.refresh());
    }

    public double getVelocity() {
        return velocity.refresh().getValueAsDouble();
    }

    public double getCurrent() {
        return current.refresh().getValueAsDouble();
    }
}

