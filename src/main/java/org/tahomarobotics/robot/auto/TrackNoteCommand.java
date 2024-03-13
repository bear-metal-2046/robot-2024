package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;

public class TrackNoteCommand extends Command {
    private Translation2d notePosition;
    private final Chassis chassis = Chassis.getInstance();

    public TrackNoteCommand() {

    }

    @Override
    public void initialize() {
        notePosition = chassis.getObjectDetectionCamera().getNotePosition().toTranslation2d();
        /*
        TODO: get current path constraints and set them to this one
        TODO: Also make the starting speed the same as current speed
         */
        AutoBuilder.pathfindToPose(new Pose2d(notePosition, Rotation2d.fromDegrees(0.0)), new PathConstraints(0, 0, 0, 0));
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
