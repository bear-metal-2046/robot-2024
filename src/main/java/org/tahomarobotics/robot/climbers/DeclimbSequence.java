package org.tahomarobotics.robot.climbers;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DeclimbSequence extends SequentialCommandGroup {
    public DeclimbSequence() {
        super(
                new ClimbCommand(ClimberConstants.TOP_POSITION, ClimberConstants.CLIMB_LADEN_SLOT),
                //TODO: Move away from chain here
                Commands.parallel(
                    new ClimbCommand(ClimberConstants.BOTTOM_POSITION, ClimberConstants.CLIMB_UNLADEN_SLOT)
                        //TODO: Return amp arm to stow here (and retract stabilizer)
                )
        );
    }
}