package org.tahomarobotics.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.shooter.Shooter;

public class ShooterDefaultCommand extends Command {

    private final Shooter shooter = Shooter.getInstance();

    public ShooterDefaultCommand() {
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (shooter.inShootingMode()) shooter.setAngle(shooter.angleToSpeaker());
    }
}
