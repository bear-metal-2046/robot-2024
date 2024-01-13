package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.util.SwerveRateLimiter;

import java.util.Arrays;
import java.util.stream.IntStream;

public class Godothething extends Command {
    private final Chassis chassis = Chassis.getInstance();

    private final Timer timer = new Timer();
    private final SwerveRateLimiter rateLimiter;

    private Double[] dists;

    public Godothething() {
        rateLimiter = new SwerveRateLimiter(
                ChassisConstants.TRANSLATION_LIMIT,
                ChassisConstants.ROTATION_LIMIT);
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        dists = Arrays.stream(chassis.getSwerveModulePositions()).map(m -> m.distanceMeters).toArray(Double[]::new);
        timer.restart();
    }

    @Override
    public void execute() {
        chassis.drive(rateLimiter.calculate(new ChassisSpeeds(0.25, 0.0, 0.0)));
    }

    @Override
    public boolean isFinished() {
        return
                chassis.getPose().getTranslation().getX() > Units.inchesToMeters(94);
    }

    @Override
    public void end(boolean interrupted) {
        var poss = chassis.getSwerveModulePositions();
        DriverStation.reportError(IntStream.range(0, 4).mapToDouble(i -> poss[i].distanceMeters - dists[i]).boxed().toList().toString(), false);
        chassis.drive(new ChassisSpeeds());
    }
}
