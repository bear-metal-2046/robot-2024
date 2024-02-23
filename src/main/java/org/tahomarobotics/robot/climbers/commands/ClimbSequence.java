package org.tahomarobotics.robot.climbers.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

import static org.tahomarobotics.robot.climbers.ClimberConstants.*;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        super(
                Commands.parallel(
                    new ClimbCommand(TOP_POSITION, UNLADEN_SLOT),
                        Commands.runOnce(() -> Shooter.getInstance().setAngle(ShooterConstants.MAX_PIVOT_ANGLE))
                        // TODO: Align with the chain here (and deploy stabilizer)
                ),
                AmpArmCommands.AMP_ARM_CLIMB.get(),
                new ClimbCommand(BOTTOM_POSITION, LADEN_SLOT)
        );
    }
}