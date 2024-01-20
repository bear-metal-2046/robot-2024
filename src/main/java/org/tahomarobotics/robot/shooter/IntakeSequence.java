package org.tahomarobotics.robot.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class IntakeSequence extends SequentialCommandGroup {
    public IntakeSequence() {
        super(
                new IntakeCommand(-0.1),
                new IndexCommand(0.5,5)
        );
    }
}
