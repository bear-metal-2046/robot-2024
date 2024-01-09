package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

public class GyroIOReal implements GyroIO {
    protected final Pigeon2 pigeon2 = new Pigeon2(RobotMap.PIGEON);
    private final StatusSignal<Double> yaw = pigeon2.getYaw();
    private final StatusSignal<Double> yawVelocity = pigeon2.getAngularVelocityZWorld();

    public GyroIOReal() {
        pigeon2.getConfigurator().apply(new Pigeon2Configuration());

        zero();

        yaw.setUpdateFrequency(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY);
        yawVelocity.setUpdateFrequency(100.0);

        // Don't update unused signals
        pigeon2.optimizeBusUtilization();
    }

    @Override
    public void logOutputs() {
        Logger.recordOutput("Chassis/Gyro/Yaw", getYaw());
    }

    @Override
    public void zero() {
        pigeon2.setYaw(0.0);
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yaw.refresh(), yawVelocity.refresh()));
    }
}
