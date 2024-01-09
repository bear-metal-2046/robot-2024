package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.Robot;

public class GyroIOSim extends GyroIOReal {
    public void simulationPeriodic(ChassisSpeeds speeds) {
        pigeon2.getSimState().addYaw(Robot.defaultPeriodSecs * Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
    }
}
