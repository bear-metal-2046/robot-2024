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
        Collector.DeploymentState deploymentState = collector.getDeploymentState();

        switch (collector.getCollectionState()) {
            case DISABLED -> {
                collector.stopCollect();

                if (isCollecting && deploymentState == Collector.DeploymentState.DEPLOYED) collector.collect();

                if (isEjecting) collector.eject();
            }
            case COLLECTING -> {
                collector.collect();

                if (indexer.hasCollected() || deploymentState == Collector.DeploymentState.STOWED) collector.stopCollect();
            }
            case EJECTING -> {
                collector.eject();

                if (isCollecting && deploymentState == Collector.DeploymentState.DEPLOYED) collector.collect();

                if (deploymentState == Collector.DeploymentState.STOWED) collector.stopCollect();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.stopCollect();
    }
}
