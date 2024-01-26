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
            case COLLECT -> {
                indexer.collect();

                if (indexer.isBeamBroken()) indexer.setState(Indexer.State.INDEXING);
            }
            case INDEXING -> indexer.index();
            case DISABLED -> {
                indexer.stop();

                if (collector.isCollecting() && !indexer.hasCollected()) indexer.setState(Indexer.State.COLLECT);
            }
        }
    }
}
