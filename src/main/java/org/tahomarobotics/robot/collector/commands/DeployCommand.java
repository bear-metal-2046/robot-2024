package org.tahomarobotics.robot.collector.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.collector.Collector;

public class DeployCommand extends Command {

    private final Collector collector;
    private final double desiredPosition;

    public DeployCommand(Collector collector, double desiredPosition) {
        this.collector = collector;
        this.desiredPosition = desiredPosition;

        addRequirements(collector);
    }

    @Override
    public void execute() {
        collector.setDeployPosition(desiredPosition);
    }

    @Override
    public boolean isFinished() {
        return collector.isAtPosition(desiredPosition);
    }

}
