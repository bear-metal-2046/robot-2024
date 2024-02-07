package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ejml.simple.SimpleMatrix;

public class SwerveDriveKinematics extends edu.wpi.first.math.kinematics.SwerveDriveKinematics {


    double posX = 0.95;
    double negX = 1;
    double posY = 1;
    double negY = 1;

    public SwerveDriveKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
    }

    @Override
    public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {

        for(int i = 0; i < moduleDeltas.length; i++) {
            var m = moduleDeltas[i];
            var c = m.angle.getCos();
            var s = m.angle.getSin();
            double x = m.distanceMeters * c;
            double y = m.distanceMeters * s;

            x *= c > 0 ? posX : negX;
            y *= s > 0 ? posY : negY;

            var dist = Math.sqrt(x * x + y * y);
            var angle = Math.atan2(y, x);

            moduleDeltas[i] = new SwerveModulePosition(dist, new Rotation2d(angle));
        }

        return super.toTwist2d(moduleDeltas);
    }
}
