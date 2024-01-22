package org.tahomarobotics.robot.collector;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class CollectorDefaultCommand extends Command {

    private final DoubleSupplier trigger;

    private final Collector collector = Collector.getInstance();

    public CollectorDefaultCommand(DoubleSupplier trigger) {
        this.trigger = trigger;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.zeroCollector();
    }

    @Override
    public void execute() {
        if (trigger.getAsDouble() > 0.0) {
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
