package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import org.tahomarobotics.robot.RobotMap;

public class SwerveModuleIOSim extends SwerveModuleIOReal {
    private final SwerveModulePosition position = new SwerveModulePosition();
    private double lastTime = Timer.getFPGATimestamp();

    public SwerveModuleIOSim(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        super(descriptor, angularOffset);
    }

    @Override
    public void processInputs(SwerveModuleIOInputs inputs) {
        super.processInputs(inputs);

        double time = Timer.getFPGATimestamp();
        position.angle = desiredState.angle;
        position.distanceMeters += desiredState.speedMetersPerSecond * (time - lastTime);
        lastTime = time;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return position;
    }

    @Override
    public SwerveModuleState getState() {
        return desiredState;
    }
}
