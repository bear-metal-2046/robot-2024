package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.chassis.ChassisConstants.DRIVE_POSITION_COEFFICIENT;

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

    private Shooter() {
        topMotor = new TalonFX(RobotMap.TOP_SHOOTER_MOTOR);
        bottomMotor = new TalonFX(RobotMap.BOTTOM_SHOOTER_MOTOR);
        indexMotor = new TalonFX(RobotMap.INDEX_ROLLER);

        topVelocity = topMotor.getVelocity();
        bottomVelocity = bottomMotor.getVelocity();
        indexPosition = indexMotor.getPosition();
        indexVelocity = indexMotor.getVelocity();

        topMotor.setControl(new CoastOut());
        bottomMotor.setControl(new CoastOut());
        indexMotor.setControl(new StaticBrake());
        bottomMotor.setControl(new Follower(topMotor.getDeviceID(), true));
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
        indexMotor.setControl(indexMotorPosition.withPosition(angle));
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
