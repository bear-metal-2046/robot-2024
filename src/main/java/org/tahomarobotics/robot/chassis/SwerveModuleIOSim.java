package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import org.tahomarobotics.robot.RobotMap;

public class SwerveModuleIOSim extends SwerveModuleIO {
    private final SwerveModulePosition position = new SwerveModulePosition();
    private double lastTime = Timer.getFPGATimestamp();

    SwerveModuleIOSim(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        super(descriptor, angularOffset);
    }

    @Override
    void processInputs(SwerveModuleIOInputs inputs) {
        super.processInputs(inputs);

        double time = Timer.getFPGATimestamp();
        position.angle = desiredState.angle;
        position.distanceMeters += desiredState.speedMetersPerSecond * (time - lastTime);
        lastTime = time;
    }

    @Override
    SwerveModulePosition getPosition() {
        return position;
    }

    @Override
    SwerveModuleState getState() {
        return desiredState;
    }
}
