package org.tahomarobotics.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.tahomarobotics.robot.util.SubsystemIF;

public class Autonomous extends SubsystemIF {
    private static final Autonomous INSTANCE = new Autonomous();

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    LoggedDashboardChooser<Command> autoChooser = PathPlannerHelper.getAutoChooser();

    public Command getSelectedAuto() {
        return autoChooser.get();
    }
}
