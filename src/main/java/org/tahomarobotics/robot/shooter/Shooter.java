package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.shooter.ShooterConstants.shooterMotorConfiguration;
import static org.tahomarobotics.robot.shooter.ShooterConstants.indexMotorConfiguration;


public class Shooter extends SubsystemIF {

    private static final Shooter INSTANCE = new Shooter();

    public static Shooter getInstance() {
        return INSTANCE;
    }

    private final TalonFX topMotor;

    private final TalonFX bottomMotor;

    private final TalonFX indexMotor;

    private final StatusSignal<Double> topVelocity;

    private final StatusSignal<Double> bottomVelocity;

    private final StatusSignal<Double> indexPosition;

    private final StatusSignal<Double> indexVelocity;

    private final VelocityVoltage topMotorVelocity = new VelocityVoltage(0.0);

    private final PositionDutyCycle indexMotorPosition = new PositionDutyCycle(0.0);

    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    private final RobustConfigurator configurator;
    private Shooter() {
        logger = LoggerFactory.getLogger("Shooter");
        configurator = new RobustConfigurator(logger);
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 5;
        mm.MotionMagicAcceleration = 10;
        mm.MotionMagicJerk = 50;

        topMotor = new TalonFX(RobotMap.TOP_SHOOTER_MOTOR);
        bottomMotor = new TalonFX(RobotMap.BOTTOM_SHOOTER_MOTOR);
        indexMotor = new TalonFX(RobotMap.INDEX_ROLLER);

        configurator.configureTalonFX(topMotor, shooterMotorConfiguration);
        configurator.configureTalonFX(bottomMotor, shooterMotorConfiguration);
        configurator.configureTalonFX(indexMotor, indexMotorConfiguration);

        topVelocity = topMotor.getVelocity();
        bottomVelocity = bottomMotor.getVelocity();
        indexPosition = indexMotor.getPosition();
        indexVelocity = indexMotor.getVelocity();

        indexMotor.setPosition(0);

        bottomMotor.setControl(new Follower(topMotor.getDeviceID(), true));
        topMotorVelocity.EnableFOC = RobotConfiguration.USING_PHOENIX_PRO;
        indexMotorPosition.EnableFOC = RobotConfiguration.USING_PHOENIX_PRO;
    }

    public double getShooterVelocity() {
        return topVelocity.refresh().getValue();
    }

    public double getIndexAngle() {
        return BaseStatusSignal.getLatencyCompensatedValue(indexPosition.refresh(), indexVelocity.refresh());
    }

    public void setShooterVelocity(double speed) {
        topMotor.setControl(topMotorVelocity.withVelocity(speed));
    }

    public void setIndexAngle(double angle) {
        indexMotor.setControl(motionMagic.withPosition(angle));
    }

    public void stopShooter(){
        topMotor.set(0.0);
        bottomMotor.set(0.0);
    }

    public void stopIndexer(){
        indexMotor.set(0.0);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                topVelocity,
                bottomVelocity,
                indexPosition
        );
    }

}
