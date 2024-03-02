package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.chassis.Chassis;


public class HitSomething extends Command {
    private final Chassis chassis = Chassis.getInstance();
    private final Timer timer = new Timer();
    private final double speed;

    public HitSomething(double speed) {
        addRequirements(this.chassis);
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
        return timer.hasElapsed(.25) && Math.abs(AmpArm.getInstance().getArmCurrent()) > 0.3;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}
