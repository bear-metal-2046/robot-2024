package org.tahomarobotics.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private final Shooter shooter = Shooter.getInstance();

    private final double power;

    public IntakeCommand(double collectorPower) {
        this.power = collectorPower;
    }

    @Override
    public void initialize() {
        shooter.setIndexVelocity(power);
    }

    @Override
    public void execute() {
        shooter.setIndexVelocity(power);
    }

    @Override
    public boolean isFinished() {
        return !shooter.getBeamBreake();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIndexer();
        System.out.println("Triped");
    }
}
