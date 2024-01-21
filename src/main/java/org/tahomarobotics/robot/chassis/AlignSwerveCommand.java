package org.tahomarobotics.robot.chassis;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;

class AlignSwerveCommand extends Command {

    private final Chassis chassis = Chassis.getInstance();
    private boolean finalized = false;

    public AlignSwerveCommand() {
        addRequirements(chassis);
    }

    private static final String FINALIZE_KEY = "Finalize";

    @Override
    public void initialize() {
        if (!RobotState.isDisabled()) {
            cancel();
        }
        SmartDashboard.putBoolean(FINALIZE_KEY, false);
        chassis.initializeCalibration();
    }

    @Override
    public void execute() {
        if (!RobotState.isDisabled()) {
            cancel();
        }
        finalized = SmartDashboard.getBoolean(FINALIZE_KEY, false);
    }

    @Override
    public boolean isFinished() {
        return finalized;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            chassis.cancelCalibration();
        } else {
            chassis.finalizeCalibration();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
