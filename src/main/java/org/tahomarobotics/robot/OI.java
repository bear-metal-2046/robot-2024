// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.amp.AmpArmConstants;
import org.tahomarobotics.robot.amp.commands.SourceIntakeCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.commands.TeleopDriveCommand;
import org.tahomarobotics.robot.climbers.ClimberConstants;
import org.tahomarobotics.robot.climbers.Climbers;
import org.tahomarobotics.robot.climbers.commands.*;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.ShooterConstants;
import org.tahomarobotics.robot.shooter.commands.RedunShootCommand;
import org.tahomarobotics.robot.shooter.commands.ShootCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import static org.tahomarobotics.robot.amp.commands.AmpArmCommands.*;

public class OI extends SubsystemIF {
    private final static OI INSTANCE = new OI();

    private static final double ROTATIONAL_SENSITIVITY = 2;
    private static final double FORWARD_SENSITIVITY = 1.1;
    private static final double DEAD_ZONE = 0.09;

    public static OI getInstance() {
        return INSTANCE;
    }

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipController = new CommandXboxController(1);

    private static final Executor rumbleExec = Executors.newFixedThreadPool(2,
            r -> {
                Thread t = new Thread(r, "RumbleThread");
                t.setDaemon(true);
                return t;
            }
    );
    private static final long RUMBLE_TIMEOUT_MS = 300;

    public OI() {
        // Disable OI periodic unless its being used.
        CommandScheduler.getInstance().unregisterSubsystem(this);

        configureBindings();
        setDefaultCommands();
    }

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }

    /**
     * Configure the button bindings for the controller(s).
     */
    private void configureBindings() {
        Chassis chassis = Chassis.getInstance();
        Collector collector = Collector.getInstance();
        Shooter shooter = Shooter.getInstance();
        AmpArm ampArm = AmpArm.getInstance();
        Climbers climbers = Climbers.getInstance();

        // Robot Heading Zeroing
        driveController.a().onTrue(Commands.runOnce(chassis::orientToZeroHeading));

        // Robot/Field Orientation
        driveController.b().onTrue(Commands.runOnce(chassis::toggleOrientation));

        //Collector up and down
        driveController.leftBumper().onTrue(Commands.runOnce(collector::toggleDeploy));

        //Shooting mode toggle
        driveController.rightBumper().onTrue(Commands.runOnce(shooter::toggleShootMode));


        manipController.povUp().onTrue(Commands.runOnce(shooter::biasUp));
        manipController.povDown().onTrue(Commands.runOnce(shooter::biasDown));
        manipController.start().onTrue(Commands.runOnce(shooter::resetBias));

        manipController.rightBumper().onTrue(Commands.runOnce(shooter::toggleRedundantShootMode));
        manipController.povLeft().onTrue(Commands.runOnce(() -> {
            shooter.enableRedundantShootMode();
            shooter.setAngle(ShooterConstants.CLOSE_REDUNDANT_ANGLE);
        } ));
        manipController.povRight().onTrue(Commands.runOnce(() -> {
            shooter.enableRedundantShootMode();
            shooter.setAngle(ShooterConstants.FAR_REDUNDANT_ANGLE);
        } ));

        manipController.b().onTrue(Commands.runOnce(shooter::toggleIdle));

        driveController.y().onTrue(AMP_ARM_CTRL);

        driveController.start().onTrue(Commands.deferredProxy(() ->
                switch (climbers.getClimbState()) {
                    case COCKED -> new PreClimbSequence();
                    case READY -> new EngageCommand();
                    default -> Commands.none();
                })
        );
        driveController.x().onTrue(Commands.deferredProxy(ClimbSequence::new).onlyIf(() -> climbers.getClimbState() == Climbers.ClimbState.ENGAGED));
        driveController.back().onTrue(Commands.deferredProxy(() ->
                switch (climbers.getClimbState()) {
                    case READY, PRE_CLIMBING -> new PreClimbCancel();
                    case ENGAGED, ENGAGING -> new EngagedCancel();
                    // Descend with break mode in the case that it doesn't work.
                    // TODO: At this point, we can't be sure of anything about the state of the robot so designating
                    //  the canceling to a separate command that does a full reset might be ideal.
                    case CLIMBING, CLIMBED -> Commands.runOnce(() -> {
                        ampArm.setRollerState(AmpArm.RollerState.DISABLED);
                        climbers.stop();
                        climbers.setClimbState(Climbers.ClimbState.ENGAGED);
                    });
                    default -> Commands.none();
                })
        );

        SmartDashboard.putData("Climbers UP", new UnladenClimbCommand(ClimberConstants.TOP_POSITION));
        SmartDashboard.putData("Climbers DOWN", Commands.sequence(
                Commands.runOnce(() -> shooter.setAngle(ShooterConstants.MAX_PIVOT_ANGLE)),
                Commands.waitUntil(shooter::isAtAngle),
                new UnladenClimbCommand(ClimberConstants.BOTTOM_POSITION)
        ));

        driveController.rightTrigger(0.5)
                .whileTrue(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.SCORE))
                .onlyIf(ampArm::isArmAtAmp))
                .onFalse(Commands.waitSeconds(0.25).andThen(
                        Commands.defer(ARM_TO_STOW, Set.of(ampArm))).onlyIf(ampArm::isArmAtAmp))
                .onTrue(new ShootCommand())
                .onTrue(new RedunShootCommand())
                .whileFalse(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)));

        driveController.leftTrigger(0.5).whileTrue(
                new SourceIntakeCommand().onlyIf(() -> !ampArm.getRollerState().equals(AmpArm.RollerState.COLLECTED)
                        && !ampArm.getRollerState().equals(AmpArm.RollerState.CENTERING)
                        && ampArm.isArmAtSource()))
                .onFalse(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED))
                        .onlyIf(() -> !ampArm.getRollerState().equals(AmpArm.RollerState.COLLECTED)
                                && !ampArm.getRollerState().equals(AmpArm.RollerState.CENTERING)
                                && ampArm.isArmAtSource()));

        driveController.leftTrigger(0.5)
                .onTrue(Commands.runOnce(() -> collector.setIsCollecting(true)))
                .onFalse(Commands.runOnce(() -> collector.setIsCollecting(false)));

        driveController.povLeft()
                .onTrue(Commands.runOnce(() -> {
                    collector.setIsEjecting(true);
                    shooter.enable();
                    ampArm.setRollerState(AmpArm.RollerState.PASSING);
                }))
                .onFalse(Commands.runOnce(() -> {
                    collector.setIsEjecting(false);
                    shooter.stop();
                    ampArm.setRollerState(AmpArm.RollerState.DISABLED);
                }));

        manipController.y().onTrue(Commands.deferredProxy(FEEDBACK));
    }

    private void setDefaultCommands() {
        Chassis.getInstance().setDefaultCommand(new TeleopDriveCommand(
                inputs -> {
                    inputs.x = -desensitizePowerBased(driveController.getLeftY(), FORWARD_SENSITIVITY);
                    inputs.y = -desensitizePowerBased(driveController.getLeftX(), FORWARD_SENSITIVITY);
                    inputs.rot = -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_SENSITIVITY);
                }
        ));
    }

    public boolean isManipXPressed() {
        return manipController.x().getAsBoolean();
    }

    private static double deadband(double value, double deadZone) {
        if (Math.abs(value) > deadZone) {
            if (value > 0.0) {
                return (value - deadZone) / (1.0 - deadZone);
            } else {
                return (value + deadZone) / (1.0 - deadZone);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Reduces the sensitivity around the zero point to make the Robot more
     * controllable.
     *
     * @param value - raw input
     * @param power - 1.0 indicates linear (full sensitivity) - larger number
     *              reduces small values
     * @return 0 to +/- 100%
     */
    private static double desensitizePowerBased(double value, double power) {
        value = deadband(value, DEAD_ZONE);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }


    // RUMBLE

    public void rumbleDrive() {
        rumble(driveController.getHID());
    }

    private void rumble(XboxController controller) {
        rumbleExec.execute(() -> {
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
            controller.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            try {
                Thread.sleep(RUMBLE_TIMEOUT_MS);
            } catch (InterruptedException e) {
                // ignore and disable
                Thread.currentThread().interrupt();
            }
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        });
    }

    public void log(String msg) {
        logger.info(msg);
    }
}