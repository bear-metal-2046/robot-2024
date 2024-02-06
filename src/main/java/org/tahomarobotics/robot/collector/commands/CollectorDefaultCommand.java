package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;

import java.util.function.Consumer;

public class CollectorDefaultCommand extends Command {

    private final CollectorDefaultCommandInputsAutoLogged inputs = new CollectorDefaultCommandInputsAutoLogged();
    private final Consumer<CollectorDefaultCommandInputs> inpMut;

    private final Collector collector = Collector.getInstance();
    private final Indexer indexer = Indexer.getInstance();

    public CollectorDefaultCommand(Consumer<CollectorDefaultCommandInputs> inpMut) {
        this.inpMut = inpMut;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        if (RobotState.isAutonomous()) {
            executeAuto();
        } else {
            executeTeleOp();
        }
    }

    private void executeTeleOp() {
        inpMut.accept(inputs);

        Logger.processInputs("Collector/ControllerInputs", inputs);

        boolean isCollecting = inputs.trigger > 0.0;
        boolean isEjecting = inputs.eject;

        switch (collector.getCollectionState()) {
            case DISABLED -> {
                collector.stopCollect();

                if (collector.isInEject()) collector.toggleDeploy();

                if (isCollecting && !collector.isStowed() && !indexer.hasCollected()) collector.setCollectionState(Collector.CollectionState.COLLECTING);

                if (isEjecting) collector.setCollectionState(Collector.CollectionState.EJECTING);
            }
            case COLLECTING -> {
                collector.collect();

                if (indexer.hasCollected()) collector.toggleDeploy();

                if ((!isCollecting && !indexer.isIndexing()) || collector.isStowed()) collector.setCollectionState(Collector.CollectionState.DISABLED);
            }
            case EJECTING -> {
                collector.eject();

                if (!collector.isStowed()) collector.setDeployEject();

                if (isCollecting && !collector.isStowed()) collector.setCollectionState(Collector.CollectionState.COLLECTING);

                if (!isEjecting) collector.setCollectionState(Collector.CollectionState.DISABLED);
            }
        }
    }

    private void executeAuto() {
        switch (collector.getCollectionState()) {
            case DISABLED -> {
                collector.stopCollect();

                if (collector.isDeployed()) collector.setCollectionState(Collector.CollectionState.COLLECTING);
            }
            case COLLECTING -> {
                collector.collect();

                if (collector.isStowed()) collector.setCollectionState(Collector.CollectionState.DISABLED);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stopCollect();
    }

    @AutoLog
    public static class CollectorDefaultCommandInputs {
        public double trigger;
        public boolean eject;
    }
}
