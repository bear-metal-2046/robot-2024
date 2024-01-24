package org.tahomarobotics.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.indexer.Indexer;

public class IndexerDefaultCommand extends Command {
    private final Indexer indexer = Indexer.getInstance();
//    private final Collector collector = Collector.getInstance();

    public IndexerDefaultCommand() {
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        // if (!collector.isRunning()) {
        //     indexer.stop();
        //     return;
        // }

        if (indexer.hasCollected()) indexer.stop();
        else if (!indexer.isBeamBroken() && !indexer.isIndexing()) indexer.collect();
        else indexer.index();
    }
}
