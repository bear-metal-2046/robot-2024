// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.amp.AmpArm;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.commands.TeleopDriveCommand;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.commands.CollectorDefaultCommand;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.indexer.commands.IndexerDefaultCommand;
import org.tahomarobotics.robot.shooter.Shooter;
import org.tahomarobotics.robot.shooter.commands.ShootCommand;
import org.tahomarobotics.robot.shooter.commands.ShooterDefaultCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.amp.commands.AmpArmCommands.AMP_ARM_CTRL;

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

    public OI() {
        // Disable OI periodic unless its being used.
        CommandScheduler.getInstance().unregisterSubsystem(this);

        configureBindings();
        setDefaultCommands();
    }

    /**
     * Configure the button bindings for the controller(s).
     */
    private void configureBindings() {
        Chassis chassis = Chassis.getInstance();
        Collector collector = Collector.getInstance();
        Shooter shooter = Shooter.getInstance();
        AmpArm ampArm = AmpArm.getInstance();

        // Robot Heading Zeroing
        driveController.a().onTrue(Commands.runOnce(chassis::orientToZeroHeading));

        // Robot/Field Orientation
        driveController.b().onTrue(Commands.runOnce(chassis::toggleOrientation));

        //Collector up and down
        driveController.leftBumper().onTrue(Commands.runOnce(collector::toggleDeploy));

        //Shooting mode toggle
        driveController.rightBumper().onTrue(Commands.runOnce(shooter::toggleShootMode));

        // Shoot
        driveController.x().onTrue(new ShootCommand());

        driveController.povUp().whileTrue(Commands.run(shooter::biasUp));
        driveController.povDown().whileTrue(Commands.run(shooter::biasDown));
        driveController.povDownLeft().onTrue(Commands.runOnce(shooter::resetBias));

        driveController.y().onTrue(AMP_ARM_CTRL);

        driveController.rightTrigger(0.5)
                .whileTrue(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.SCORE))
                .onlyIf(ampArm::isAmp))
                .whileFalse(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED)));

        driveController.leftTrigger(0.01)
                .whileTrue(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.PASSING))
                .onlyIf(ampArm::isSource))
                .whileFalse(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.DISABLED))
                .onlyIf(ampArm::isSource))
                .onFalse(Commands.runOnce(() -> ampArm.setRollerState(AmpArm.RollerState.COLLECTED)));
    }

    private void setDefaultCommands() {
        Chassis.getInstance().setDefaultCommand(new TeleopDriveCommand(
                inputs -> {
                    inputs.x = -desensitizePowerBased(driveController.getLeftY(), FORWARD_SENSITIVITY);
                    inputs.y = -desensitizePowerBased(driveController.getLeftX(), FORWARD_SENSITIVITY);
                    inputs.rot = -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_SENSITIVITY);
                }
        ));

        Collector.getInstance().setDefaultCommand(new CollectorDefaultCommand(
                inputs -> {
                    inputs.trigger = deadband(driveController.getLeftTriggerAxis(), 0.5);
                    inputs.eject = driveController.povLeft().getAsBoolean();
                }
        ));

        Indexer.getInstance().setDefaultCommand(new IndexerDefaultCommand());

        Shooter.getInstance().setDefaultCommand(new ShooterDefaultCommand());
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
}