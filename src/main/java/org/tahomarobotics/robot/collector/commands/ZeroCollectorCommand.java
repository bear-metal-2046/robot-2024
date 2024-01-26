package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;

public class ZeroCollectorCommand extends Command {

    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ZeroCollectorCommand.class);

    private final Collector collector = Collector.getInstance();

    private static final double TIMEOUT = 5;

    private static final double STOPPED_VELOCITY_THRESHOLD = 0.01;
    private static final double INITIAL_MOVE_TIME = 0.1;

    private final Timer timer = new Timer();

    public ZeroCollectorCommand() {
        addRequirements(this.collector);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        collector.setVoltage(-0.5);
    }

    @Override
    public boolean isFinished() {
        return hasStopped() || timer.hasElapsed(TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        logger.info("ZEROED COLLECTOR");
        collector.stopDeploy();
        collector.zeroCollector();
    }

    private boolean hasStopped() {
        return timer.hasElapsed(INITIAL_MOVE_TIME) && Math.abs(collector.getDeployVelocity()) < STOPPED_VELOCITY_THRESHOLD;
    }
}
