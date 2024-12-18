package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import java.util.List;

public class GyroIO {
    protected final Pigeon2 pigeon2 = new Pigeon2(RobotMap.PIGEON, RobotConfiguration.CANBUS_NAME);
    private final StatusSignal<Double> yaw = pigeon2.getYaw();
    private final StatusSignal<Double> yawVelocity = pigeon2.getAngularVelocityZWorld();

    public record ValidYaw(Rotation2d yaw, boolean valid){}

    GyroIO() {
        pigeon2.getConfigurator().apply(new Pigeon2Configuration());

        zero();

        yaw.setUpdateFrequency(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY);
        yawVelocity.setUpdateFrequency(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY);

        // Don't update unused signals
        pigeon2.optimizeBusUtilization();
    }

    void zero() {
        pigeon2.setYaw(0.0);
    }

    ValidYaw getYaw() {
        boolean valid = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        return new ValidYaw(Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yaw, yawVelocity)), valid);
    }

    List<BaseStatusSignal> getStatusSignals() {
        return List.of(yaw, yawVelocity);
    }
}
