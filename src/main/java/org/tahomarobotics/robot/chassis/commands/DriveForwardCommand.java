package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;


public class DriveForwardCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();
    private Pose2d targetPose;
    private final double distance_m;
    private final double speed;

    public DriveForwardCommand(double distance, double speed) {
        addRequirements(this.chassis);
        this.distance_m = distance;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        targetPose = Chassis.getInstance().getPose().plus(new Transform2d(distance_m, 0.0, new Rotation2d()));
    }

    @Override
    public void execute() {
        chassis.drive(new ChassisSpeeds(speed, 0, 0), false);
    }

    @Override
    public boolean isFinished() {
        return targetPose.getTranslation().getDistance(chassis.getPose().getTranslation()) < 0.03;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
