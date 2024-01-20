package org.tahomarobotics.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCommand extends Command {
    private final Shooter shooter = Shooter.getInstance();

    private double velocity;
    private final double time;
    private final Timer timer = new Timer();

    public ShootCommand(double time) {
        this.time = time;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        velocity = SmartDashboard.getNumber("Shooter RPM", 5000.0);
//        velocity = 5000;
        timer.restart();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Shoot Veolocity", shooter.getShooterVelocity());
        shooter.setShooterVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}
