package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;

import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.IntStream;

public class KnownMovementCommand extends Command {
    private static final Logger logger = LoggerFactory.getLogger(KnownMovementCommand.class);

    private final Chassis chassis = Chassis.getInstance();

    //    private final SwerveRateLimiter limiter = new SwerveRateLimiter(ChassisConstants.TRANSLATION_LIMIT, ChassisConstants.ROTATION_LIMIT);
    private SwerveModulePosition[] initialPositions;

    private final ChassisSpeeds speeds;
    private final Function<Pose2d, Boolean> endCondition;

    public KnownMovementCommand(double x, double y, double r, Function<Pose2d, Boolean> endCondition) {
        this.speeds = new ChassisSpeeds(x, y, r);
        this.endCondition = endCondition;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
//        chassis.zeroPose();
        logger.info("Known Movement Command initialized");
        initialPositions = chassis.getSwerveModulePositions();
    }

    @Override
    public void execute() {
        chassis.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return !endCondition.apply(chassis.getPose());
    }

    @Override
    public void end(boolean interrupted) {
        logger.info("Known Movement Command stopped");

        chassis.drive(new ChassisSpeeds());

        var currentPositions = chassis.getSwerveModulePositions();
        DriverStation.reportError("Distances traveled: " + Arrays.toString(IntStream.range(0, 4).mapToDouble(i -> currentPositions[i].distanceMeters - initialPositions[i].distanceMeters).boxed().toArray()), false);
    }
}