package org.tahomarobotics.robot.climbers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SysIdTest;

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

    public void zeroAtCurrentPosition() {
        climbMotor.setPosition(0);
    }

    public void setSlotTuning(Slot0Configs slotConfig) {
        climbMotor.getConfigurator().apply(slotConfig);
    }

    public void setTargetPos(double targetPosition) {
        climbMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));
    }

    public TalonFX getMotor() {
        return climbMotor;
    }

    public double getPosition() {
        return climbMotor.getPosition().refresh().getValue();
    }

    public double getVelocity() {
        return climbMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        climbMotor.stopMotor();
    }

    public void runWithVelocity(double targetVelocity) {
        climbMotor.setControl(velocityControl.withVelocity(targetVelocity).withSlot(0));
    }
}

