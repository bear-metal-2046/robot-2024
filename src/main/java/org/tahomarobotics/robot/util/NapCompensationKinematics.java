package org.tahomarobotics.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.Logger;

public class NapCompensationKinematics extends SwerveDriveKinematics {

    public NapCompensationKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
    }

    @Override
    public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {
        var twist = super.toTwist2d(moduleDeltas);

        Logger.recordOutput("Twist dy (before)", twist.dx);
        if (twist.dx > 0) twist.dx *= 0.95;
        Logger.recordOutput("Twist dy (after)", twist.dx);

        return twist;
    }
}
