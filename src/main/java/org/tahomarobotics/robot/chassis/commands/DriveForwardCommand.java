package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;


public class DriveForwardCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();
    private Pose2d initialPose;
    private final double distance_m;
    private final double speed;

    public DriveForwardCommand(double distance, double speed) {
        addRequirements(this.chassis);
        this.distance_m = distance;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        initialPose = chassis.getPose();
    }

    @Override
    public void execute() {
        chassis.drive(new ChassisSpeeds(0, speed, 0), false);
    }

    @Override
    public boolean isFinished() {
        return initialPose.getTranslation().getDistance(chassis.getPose().getTranslation()) > distance_m;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
