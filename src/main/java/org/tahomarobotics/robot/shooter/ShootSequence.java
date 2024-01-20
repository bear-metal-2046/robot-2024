package org.tahomarobotics.robot.shooter;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootSequence extends ParallelCommandGroup {
    public ShootSequence() {
        super(
                new ShootCommand(5),
                new SequentialCommandGroup(
                        Commands.waitUntil(() -> Shooter.getInstance().getShooterVelocity() > SmartDashboard.getNumber("Shooter RPM", 5000) - 1000),
                        Commands.print("SHOOT"),
                        new IndexCommand(0.5,5)
                )
        );
    }
}