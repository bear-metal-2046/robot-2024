package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;

import java.util.function.DoubleSupplier;

public class CollectorDefaultCommand extends Command {

    private final DoubleSupplier trigger;

    private final Collector collector = Collector.getInstance();
    private final Indexer indexer = Indexer.getInstance();

    public CollectorDefaultCommand(DoubleSupplier trigger) {
        this.trigger = trigger;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        if (trigger.getAsDouble() > 0.0 && !collector.isStowed() && !indexer.hasCollected()) {
            collector.collect();
        } else {
            collector.stopCollect();
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stopCollect();
    }
}
