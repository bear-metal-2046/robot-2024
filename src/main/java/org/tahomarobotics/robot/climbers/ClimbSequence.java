package org.tahomarobotics.robot.climbers;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        super(
                Commands.parallel(
                    new ClimbCommand(ClimberConstants.TOP_POSITION, ClimberConstants.CLIMB_UNLADEN_SLOT)
                        // TODO: Align with the chain here (and deploy stabilizer)
                ),
                // TODO: Swing amp arm here
                new ClimbCommand(ClimberConstants.BOTTOM_POSITION, ClimberConstants.CLIMB_LADEN_SLOT)
        );
    }
}