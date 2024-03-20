package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveLimiter {


    private SwerveModuleState[] prevStates;
    private double prevTime;

    private final double accelerationLimit;

    public SwerveDriveLimiter(SwerveModuleState[] states, double accelerationLimit) {
        prevStates = states;
        this.accelerationLimit = accelerationLimit;
        prevTime = MathSharedStore.getTimestamp();
    }
    public SwerveModuleState[] calculate(SwerveModuleState states[]) {

        double acceleration[] = new double[states.length];
        double currentTime = MathSharedStore.getTimestamp();
        double dT = currentTime - prevTime;
        prevTime = currentTime;

        double averageAcceleration = 0;
        for (int i = 0; i < states.length; i++) {
            acceleration[i] = (states[i].speedMetersPerSecond -  prevStates[i].speedMetersPerSecond) / dT;
            averageAcceleration += Math.abs(acceleration[i]);
        }
        averageAcceleration /= states.length;

        double scale = averageAcceleration > accelerationLimit ?  accelerationLimit/averageAcceleration : 1.0;

        for(int i = 0; i < states.length; i++) {
            states[i].speedMetersPerSecond = prevStates[i].speedMetersPerSecond + scale * acceleration[i] * dT;
        }

        prevStates = states;
        return states;
    }
}
