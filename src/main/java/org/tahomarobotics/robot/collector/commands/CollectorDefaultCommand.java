package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CollectorDefaultCommand extends Command {

    private final DoubleSupplier trigger;
    private final BooleanSupplier povLeft;

    private final Collector collector = Collector.getInstance();
    private final Indexer indexer = Indexer.getInstance();

    public CollectorDefaultCommand(DoubleSupplier trigger, BooleanSupplier povLeft) {
        this.trigger = trigger;
        this.povLeft = povLeft;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        boolean isCollecting = trigger.getAsDouble() > 0.0;
        boolean isEjecting = povLeft.getAsBoolean();

        switch (collector.getCollectionState()) {
            case DISABLED -> {
                collector.stopCollect();

                if (isCollecting && !collector.isStowed() && !indexer.hasCollected()) collector.setCollectionState(Collector.CollectionState.COLLECTING);

                if (isEjecting) collector.setCollectionState(Collector.CollectionState.EJECTING);
            }
            case COLLECTING -> {
                collector.collect();

                if (!isCollecting || indexer.hasCollected() || collector.isStowed()) collector.setCollectionState(Collector.CollectionState.DISABLED);
            }
            case EJECTING -> {
                collector.eject();

                if (isCollecting && !collector.isStowed()) collector.setCollectionState(Collector.CollectionState.COLLECTING);

                if (!isEjecting) collector.stopCollect();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stopCollect();
    }
}
