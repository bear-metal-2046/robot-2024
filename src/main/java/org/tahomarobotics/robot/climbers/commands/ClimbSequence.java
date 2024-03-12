package org.tahomarobotics.robot.climbers.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.OI;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.Climbers;

import static org.tahomarobotics.robot.climbers.ClimberConstants.ALMOST_BOTTOM_POSITION;
import static org.tahomarobotics.robot.climbers.ClimberConstants.BOTTOM_POSITION;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        Climbers climbers = Climbers.getInstance();
        AmpArm ampArm = AmpArm.getInstance();

        addCommands(
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.CLIMBING)),
                new LadenClimbCommand(BOTTOM_POSITION),
                AmpArmCommands.ARM_TO_TRAP.get(),
                Commands.waitSeconds(0.25),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.TRAP)),
                Commands.waitUntil(OI.getInstance()::isManipXPressed).raceWith(Commands.waitSeconds(5)),
                new LadenClimbCommand(ALMOST_BOTTOM_POSITION),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.CLIMBED))
        );
    }
}