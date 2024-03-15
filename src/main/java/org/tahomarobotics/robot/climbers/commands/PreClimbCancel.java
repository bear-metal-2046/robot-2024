package org.tahomarobotics.robot.climbers.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.amp.commands.AmpArmCommands;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;

public class PreClimbCancel extends SequentialCommandGroup {
    private static final Logger logger = LoggerFactory.getLogger(PreClimbCancel.class);

    public PreClimbCancel() {
        Climbers climbers = Climbers.getInstance();

        addCommands(Commands.runOnce(() -> logger.info("Canceled Pre-Climb")),
                Commands.runOnce(() -> Shooter.getInstance().setAngle(ShooterConstants.MAX_PIVOT_ANGLE)),
                Commands.waitUntil(Shooter.getInstance()::isAtAngle),
                new UnladenClimbCommand(ClimberConstants.BOTTOM_POSITION)
        );

        if (climbers.isTrapping())
            addCommands(
                    AmpArmCommands.ARM_TO_STOW.get(),
                    AmpArmCommands.FEEDBACK.get()
            );

        addCommands(
                Commands.runOnce(() -> climbers.setClimbState(Climbers.ClimbState.COCKED))
        );
    }
}
