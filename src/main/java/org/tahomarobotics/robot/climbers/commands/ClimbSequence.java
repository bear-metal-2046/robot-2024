package org.tahomarobotics.robot.climbers.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.climbers.Climbers;

import static org.tahomarobotics.robot.amp.AmpArmConstants.*;
import static org.tahomarobotics.robot.climbers.ClimberConstants.BOTTOM_POSITION;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        Climbers climbers = Climbers.getInstance();
        AmpArm ampArm = AmpArm.getInstance();

        addCommands(
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.CLIMBING)),
                new LadenClimbCommand(BOTTOM_POSITION),
                Commands.runOnce(() -> ampArm.setRollerPosition(NOTE_INTAKE_POSITION)),
                Commands.waitUntil(ampArm::isRollerAtPosition),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)),
                Commands.runOnce(() -> ampArm.setArmPosition(ARM_TRAP_POSE)),
                Commands.waitUntil(ampArm::isArmAtPosition),
                Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.TRAP)),
                Commands.runOnce(() -> AmpArm.getInstance().setWristPosition(WRIST_TRAP_POSE)),
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.CLIMBED))
        );
    }
}