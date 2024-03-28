//package org.tahomarobotics.robot.shooter.commands;
//
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.Command;
//import org.slf4j.LoggerFactory;
//import org.tahomarobotics.robot.shooter.Shooter;
//
//public class ZeroShooterCommand extends Command {
//
//    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ZeroShooterCommand.class);
//
//    private final Shooter shooter = Shooter.getInstance();
//
//    private static final double TIMEOUT = 5;
//
//    private static final double STOPPED_VELOCITY_THRESHOLD = 0.01;
//    private static final double INITIAL_MOVE_TIME = 0.1;
//
//    private final Timer timer = new Timer();
//
//    public ZeroShooterCommand() {
//        addRequirements(shooter);
//    }
//
//    @Override
//    public void initialize() {
//        logger.info("Zeroing Collector");
//        timer.restart();
//    }
//
//    @Override
//    public void execute() {
//        shooter.setPivotVoltage(-0.5);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return hasStopped() || timer.hasElapsed(TIMEOUT);
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        logger.info("ZEROED SHOOTER");
//        shooter.setPivotVoltage(0.0);
//        shooter.zero();
//    }
//
//    private boolean hasStopped() {
//        return timer.hasElapsed(INITIAL_MOVE_TIME) && Math.abs(shooter.getPivotVelocity()) < STOPPED_VELOCITY_THRESHOLD;
//    }
//}
