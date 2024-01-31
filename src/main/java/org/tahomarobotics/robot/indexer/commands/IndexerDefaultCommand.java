package org.tahomarobotics.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;

public class IndexerDefaultCommand extends Command {
    private static final Logger logger = LoggerFactory.getLogger(IndexerDefaultCommand.class);

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

                if (collector.isCollecting() && !indexer.hasCollected()) indexer.transitionToCollecting();
                if (collector.isEjecting()) indexer.transitionToEjecting();
            }
            case COLLECTING -> {
                indexer.collect();

                if (!collector.isCollecting()) indexer.transitionToDisabled();
                if (indexer.isBeamBrokenOne()) indexer.transitionToIndexing();
                if (collector.isEjecting()) indexer.transitionToEjecting();
            }
            case INDEXING -> {
                indexer.index();

                if (collector.isEjecting()) indexer.transitionToEjecting();
            }
            case EJECTING -> {
                indexer.eject();

                if (!collector.isEjecting()) indexer.transitionToDisabled();
            }
            case TRANSFERRING -> {
                indexer.transfer();

                if (!indexer.isBeamBrokenOne()) indexer.transitionToDisabled();
            }
            case COLLECTED -> {
                // Fix possible broken state
                if (!indexer.isBeamBrokenOne()) {
                    logger.error("Indexer in COLLECTED state with no note detected! Transitioning to DISABLED...");
                    indexer.transitionToDisabled();
                }
                if (collector.isEjecting()) indexer.transitionToEjecting();
            }
        }
    }
}
