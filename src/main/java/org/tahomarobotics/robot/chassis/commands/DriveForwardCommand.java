package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;


public class DriveForwardCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();
    private final Timer timer = new Timer();
    private final double distance_m;
    private final double speed;

    public DriveForwardCommand(double distance, double speed) {
        addRequirements(this.chassis);
        this.distance_m = distance;
        this.speed = speed;
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
        return timer.hasElapsed(distance_m / speed);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}
