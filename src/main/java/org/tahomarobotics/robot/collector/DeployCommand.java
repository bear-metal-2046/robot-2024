package org.tahomarobotics.robot.collector;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployCommand extends Command {

    private final Collector collector = Collector.getInstance();
    private final double desiredPosition;

    public DeployCommand(Collector collector, double desiredPosition) {
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
