/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * <p>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * <p>
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 * <p>
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 * <p>
 */
package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.chassis.commands.TeleopDriveCommandInputsAutoLogged;
import org.tahomarobotics.robot.util.SwerveRateLimiter;

import java.util.function.Consumer;

class TeleopDriveCommand extends Command {

    private static final Chassis chassis = Chassis.getInstance();

    private final ChassisSpeeds velocityInput = new ChassisSpeeds();

    private final SwerveRateLimiter rateLimiter;

    private final TeleopDriveCommandInputsAutoLogged inputs = new TeleopDriveCommandInputsAutoLogged();
    private final Consumer<TeleopDriveCommandInputsAutoLogged> inpMut;

    private final double maxVelocity;
    private final double maxRotationalVelocity;

    public TeleopDriveCommand(Consumer<TeleopDriveCommandInputsAutoLogged> inpMut) {
        this.inpMut = inpMut;

        addRequirements(chassis);

        rateLimiter = new SwerveRateLimiter(
                ChassisConstants.TRANSLATION_LIMIT,
                ChassisConstants.ROTATION_LIMIT);

        maxVelocity = ChassisConstants.MAX_VELOCITY;
        maxRotationalVelocity = ChassisConstants.MAX_ANGULAR_VELOCITY;
    }

    @Override
    public void execute() {
        inpMut.accept(inputs);
        Logger.processInputs("Chassis/ControllerInputs", inputs);

        double direction = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red ? -1.0 : 1.0;

        velocityInput.vxMetersPerSecond = inputs.x * maxVelocity * direction;
        velocityInput.vyMetersPerSecond = inputs.y * maxVelocity * direction;
        velocityInput.omegaRadiansPerSecond = inputs.rot * maxRotationalVelocity;

        ChassisSpeeds velocityOutput = rateLimiter.calculate(velocityInput);

        chassis.drive(velocityOutput);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }

    @AutoLog
    public static class TeleopDriveCommandInputs {
        public double x, y, rot;
    }
}
