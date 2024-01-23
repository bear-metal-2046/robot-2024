// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.commands.TeleopDriveCommand;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.collector.CollectorDefaultCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

public class OI extends SubsystemIF {
    private final static OI INSTANCE = new OI();

    private static final double ROTATIONAL_SENSITIVITY = 2;
    private static final double FORWARD_SENSITIVITY = 1.1;
    private static final double COLLECT_SENSITIVITY = 1.0;
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

        // Robot Heading Zeroing
        driveController.a().onTrue(Commands.runOnce(chassis::orientToZeroHeading));
        // Robot/Field Orientation
        driveController.b().onTrue(Commands.runOnce(chassis::toggleOrientation));

        driveController.leftBumper().toggleOnTrue(collector.getDeployCommand(CollectorConstants.COLLECT_POSITION));
        driveController.leftBumper().toggleOnFalse(collector.getDeployCommand(CollectorConstants.STOW_POSITION));

//        Code for testing odometry
//        driveController.x().onTrue(Commands.runOnce(chassis::zeroPose));
//
//        driveController.povLeft().onTrue(new KnownMovementCommand(0.5, 0.0, 0.0, p -> p.getTranslation().getX() < 2.0)
//                .andThen(new KnownMovementCommand(0.0, 0.5, 0.0, p -> p.getTranslation().getY() < 2.0)
//                .andThen(new KnownMovementCommand(-0.5, 0.0, 0.0, p -> p.getTranslation().getX() > 0.0)
//                .andThen(new KnownMovementCommand(0.0, -0.5, 0.0, p -> p.getTranslation().getY() > 0.0)))));
    }

    private void setDefaultCommands() {
        Chassis chassis = Chassis.getInstance();

        chassis.setDefaultCommand(new TeleopDriveCommand(
                inputs -> {
                    inputs.x = -desensitizePowerBased(driveController.getLeftY(), FORWARD_SENSITIVITY);
                    inputs.y = -desensitizePowerBased(driveController.getLeftX(), FORWARD_SENSITIVITY);
                    inputs.rot = -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_SENSITIVITY);
                }
        ));

        Collector collector = Collector.getInstance();

        collector.setDefaultCommand(new CollectorDefaultCommand(
                () -> desensitizePowerBased(driveController.getLeftTriggerAxis(), COLLECT_SENSITIVITY)
        ));
    }

    private static double deadband(double value) {
        if (Math.abs(value) > OI.DEAD_ZONE) {
            if (value > 0.0) {
                return (value - OI.DEAD_ZONE) / (1.0 - OI.DEAD_ZONE);
            } else {
                return (value + OI.DEAD_ZONE) / (1.0 - OI.DEAD_ZONE);
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
        value = deadband(value);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}