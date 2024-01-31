package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.util.RobustConfigurator;

class Climber{
    private final Logger logger;
    private final TalonFX climbMotor;
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    public Climber(int motorID, String name) {
        logger = LoggerFactory.getLogger(name);

        RobustConfigurator configurator = new RobustConfigurator(logger);
        climbMotor = new TalonFX(motorID);
        configurator.configureTalonFX(climbMotor, ClimberConstants.CLIMB_CONFIGURATION);
    }

    public void zeroPosition() {
        climbMotor.setPosition(0);
    }

    public void setSlotTuning(Slot0Configs slotConfig) {
        climbMotor.getConfigurator().apply(slotConfig);
    }

    public void setTargetPos(double targetPositionMeters) {
        climbMotor.setControl(positionControl.withPosition(targetPositionMeters * ClimberConstants.POSITION_COEFFICIENT).withSlot(0));
    }

    public double getPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    public void runWithVelocity(double targetVelocityMps) {
        climbMotor.setControl(velocityControl.withVelocity(targetVelocityMps * ClimberConstants.POSITION_COEFFICIENT).withSlot(0));
    }
}

