package org.tahomarobotics.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexCommand extends Command {
    private final Shooter shooter = Shooter.getInstance();

    private final Timer timer = new Timer();
    private final double time;
    private final double distance;

    public IndexCommand(double distance, double timeout) {
        time = timeout;
        this.distance = distance;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setIndexAngle(distance);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIndexer();
        System.out.println("Endeed");
    }
}
