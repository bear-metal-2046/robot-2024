package org.tahomarobotics.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;

public class IndexerDefaultCommand extends Command {
    private final Indexer indexer = Indexer.getInstance();
    private final Collector collector = Collector.getInstance();

    public IndexerDefaultCommand() {
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        switch (indexer.getState()) {
            case DISABLED -> {
                indexer.disable();

                if (collector.isCollecting() && !indexer.hasCollected()) indexer.setState(Indexer.State.COLLECT);
                if (collector.isEjecting()) indexer.setState(Indexer.State.EJECTING);
            }
            case COLLECT -> {
                indexer.collect();

                if (indexer.isBeamBroken()) {
                    indexer.setState(Indexer.State.INDEXING);
                    indexer.zero();;
                }
                if (collector.isEjecting()) indexer.setState(Indexer.State.EJECTING);
            }
            case INDEXING -> {
                indexer.index();

                if (collector.isEjecting()) indexer.setState(Indexer.State.EJECTING);
            }
            case EJECTING -> {
                indexer.eject();

                if (!collector.isEjecting()) indexer.setState(Indexer.State.DISABLED);
            }
            case TRANSFERRING -> {
                indexer.transfer();

                if (!indexer.isBeamBroken()) indexer.setState(Indexer.State.DISABLED);
            }
        }
    }
}
