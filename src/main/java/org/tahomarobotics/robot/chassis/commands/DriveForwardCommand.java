package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;

public class DriveForwardCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();
    private final Timer timer = new Timer();
    private final double speed, time;

    public DriveForwardCommand(double speed, double time) {
        addRequirements(this.chassis);
        this.speed = speed;
        this.time = time;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        chassis.drive(new ChassisSpeeds(speed, 0, 0), false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}
