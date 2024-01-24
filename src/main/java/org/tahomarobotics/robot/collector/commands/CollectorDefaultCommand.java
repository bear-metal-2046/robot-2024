package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;

import java.util.function.DoubleSupplier;

public class CollectorDefaultCommand extends Command {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(CollectorDefaultCommand.class);


    private final DoubleSupplier trigger;

    private final Collector collector = Collector.getInstance();

    public CollectorDefaultCommand(DoubleSupplier trigger) {
        this.trigger = trigger;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        if (trigger.getAsDouble() > 0.0) {
            collector.collect();
            logger.info("Collecting");
        } else {
            collector.stopCollect();
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stopCollect();
    }
}
