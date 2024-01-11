// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.BuildConstants;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;

public class Robot extends LoggedRobot {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(Robot.class);

    @SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
    private final List<SubsystemIF> subsystems = new ArrayList<>();

    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);

        initializeAKit();

        subsystems.add(OI.getInstance().initialize());
        subsystems.add(Chassis.getInstance().initialize());

        logger.info("Robot Initialized.");
    }

    @SuppressWarnings("DataFlowIssue")
    private void initializeAKit() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
            case 0 -> "All changes committed";
            case 1 -> "Uncommitted changes";
            default -> "Unknown";
        });

        switch (RobotConfiguration.getMode()) {
            case REAL, SIM -> {
                Logger.addDataReceiver(new WPILOGWriter()); // Write to a USB drive ("/U/logs" or "logs")
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.start();
    }
    
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void disabledInit() {
        subsystems.forEach(SubsystemIF::onDisabledInit);
    }


    @Override
    public void disabledPeriodic() {}


    @Override
    public void autonomousInit() {
        subsystems.forEach(SubsystemIF::onAutonomousInit);
        Chassis.getInstance().getAutoChooser().getSelected().schedule();

        logger.info("-=-=-=- AUTONOMOUS initiated -=-=-=-");
    }
    
    
    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {
        subsystems.forEach(SubsystemIF::onTeleopInit);

        logger.info("-=-=-=- TELEOP initiated -=-=-=-");
    }
    
    
    @Override
    public void teleopPeriodic() {}


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        logger.info("-=-=-=- TEST initiated -=-=-=-");
    }
    
    
    @Override
    public void testPeriodic() {}

    @Override
    public void simulationPeriodic() {}
}
