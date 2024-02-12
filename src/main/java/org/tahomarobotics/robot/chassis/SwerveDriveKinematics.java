package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Supplier;

public class SwerveDriveKinematics extends edu.wpi.first.math.kinematics.SwerveDriveKinematics {


    double posX = 1;
    static final double negX = 1.09;//0.95;
    double posY = 1;
    static final double negY = 1.09;//1.098;
    Supplier<Rotation2d> gyroSupplier;

    public SwerveDriveKinematics(Supplier<Rotation2d> gyroSupplier, Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
        this.gyroSupplier = gyroSupplier;
    }

    @Override
    public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {

        for(int i = 0; i < moduleDeltas.length; i++) {
            var m = moduleDeltas[i];
            var g = gyroSupplier.get();
            var fr = m.angle.plus(g);
            var c = fr.getCos();
            var s = fr.getSin();
            double x = m.distanceMeters * c;
            double y = m.distanceMeters * s;

            x *= c > 0 ? posX : negX;
            y *= s > 0 ? posY : negY;

            var dist = Math.sqrt(x * x + y * y);
            var angle = Math.atan2(y, x) - g.getRadians();

            moduleDeltas[i] = new SwerveModulePosition(dist, new Rotation2d(angle));
        }

        return super.toTwist2d(moduleDeltas);
    }
}
