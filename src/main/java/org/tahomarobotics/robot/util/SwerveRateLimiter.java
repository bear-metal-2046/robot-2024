/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
package org.tahomarobotics.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Consumer;

public class SwerveRateLimiter {

    private final double accelerationLimit;
    private final SlewRateLimiter angularRateLimiter;

    private final ChassisSpeeds output = new ChassisSpeeds();
    private double previousTime = 0;

    private final Consumer<ChassisSpeeds> currentSpeedInput;

    public SwerveRateLimiter(double accelerationLimit, double angularAccelerationLimit, Consumer<ChassisSpeeds> currentSpeedInput) {
        this.accelerationLimit = accelerationLimit;
        angularRateLimiter = new SlewRateLimiter(angularAccelerationLimit);
        this.currentSpeedInput = currentSpeedInput;
    }

    protected double getAccelerationLimit(ChassisSpeeds input) {
        return accelerationLimit;
    }

    public ChassisSpeeds calculate(ChassisSpeeds input) {

        if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) == 0.0) {
            output.vxMetersPerSecond = input.vxMetersPerSecond;
            output.vyMetersPerSecond = input.vyMetersPerSecond;
            output.omegaRadiansPerSecond = input.omegaRadiansPerSecond;
            return output;
        }
        // calculate elapsed time
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - previousTime;
        previousTime = currentTime;

        // determine delta velocities
        double dx = input.vxMetersPerSecond - output.vxMetersPerSecond;
        double dy = input.vyMetersPerSecond - output.vyMetersPerSecond;

        // convert to polar coordinates
        double dir = Math.atan2(dy, dx);
        double mag = Math.sqrt(dx * dx + dy * dy);

        // limit delta speed
        double maxDeltaSpeed = getAccelerationLimit(input) * elapsedTime;
        if (mag > maxDeltaSpeed) {
            mag = maxDeltaSpeed;

            // if reversing direction and not rotating then apply brake mode
            currentSpeedInput.accept(output);
            double currentDirection = Math.atan2(output.vyMetersPerSecond, output.vxMetersPerSecond);

            if (Math.cos(dir-currentDirection) < 0 && Math.abs(input.omegaRadiansPerSecond) < 0.1) {

                // may need to feather this deceleration
                return new ChassisSpeeds();
            }
        }

        // add delta velocity to output
        output.vxMetersPerSecond += mag * Math.cos(dir);
        output.vyMetersPerSecond += mag * Math.sin(dir);

        output.omegaRadiansPerSecond = angularRateLimiter.calculate(input.omegaRadiansPerSecond);

        return output;
    }

}
