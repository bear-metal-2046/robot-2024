package org.tahomarobotics.robot.climbers.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.ClimberConstants;

public class DeclimbSequence extends SequentialCommandGroup {
    public DeclimbSequence() {
        super(
                new ClimbCommand(ClimberConstants.TOP_POSITION, ClimberConstants.LADEN_SLOT),
                //TODO: Move away from chain here
                Commands.parallel(
                    new ClimbCommand(ClimberConstants.BOTTOM_POSITION, ClimberConstants.UNLADEN_SLOT),
                        AmpArmCommands.ARM_TO_STOW.get()
                        //TODO: Return retrct stabilizer here
                )
        );
    }
}