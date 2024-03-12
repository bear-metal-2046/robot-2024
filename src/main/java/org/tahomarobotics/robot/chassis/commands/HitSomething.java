package org.tahomarobotics.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.chassis.Chassis;


public class HitSomething extends Command {
    private static final Logger logger = LoggerFactory.getLogger(HitSomething.class);
    private final Chassis chassis = Chassis.getInstance();
    private final Timer timer = new Timer();
    private final double speed;

    public HitSomething(double speed) {
        addRequirements(this.chassis);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        logger.info("Hit something initialized");
        timer.restart();
    }

    @Override
    public void execute() {
        chassis.drive(new ChassisSpeeds(speed, 0, 0), false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(.25) && Math.abs(AmpArm.getInstance().getArmCurrent()) > 0.3;
    }

    @Override
    public void end(boolean interrupted) {
        logger.info("Robot has hit something");
        chassis.drive(new ChassisSpeeds());
    }
}
