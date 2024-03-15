package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Supplier;

/**
 * Adjust the module delta for carpet nap.  This determines which carpet half the module is on and uses
 * the compensation value for the direction the wheel is travelling.
 */
public class NapCorrectingKinematics extends SwerveDriveKinematics {

    private final Supplier<Rotation2d> gyroSupplier;
    private final Supplier<Pose2d> poseSupplier;

    private final Chassis chassis = Chassis.getInstance();

    private record Compensation (double posX, double negX, double posY, double negY){}

    private Compensation sourceSideFieldCompensation;
    private Compensation ampSideFieldCompensation;

    private final Translation2d[] modules;
    private static final double centerLine = 4.1056179;

    public NapCorrectingKinematics(Supplier<Rotation2d> gyroSupplier, Supplier<Pose2d> poseSupplier, Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
        modules = moduleTranslationsMeters;
        this.gyroSupplier = gyroSupplier;
        this.poseSupplier = poseSupplier;

//      this is for feild house carpet (Only if Pos X and Neg X are perfect @ 1.0 compensation)
        sourceSideFieldCompensation = new Compensation(1.0, 1.0, 0.976, 1.0);
        ampSideFieldCompensation = new Compensation(1.0, 1.0, 0.976, 1.0);
    }



    @Override
    public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {

        Pose2d robotPose = poseSupplier.get();

        for(int i = 0; i < moduleDeltas.length; i++) {

            // adjust module angle to field-oriented
            var m = moduleDeltas[i];
            var g = chassis.getPose().getRotation();
            var fr = m.angle.plus(g);

            // create directional components in x and y
            var c = fr.getCos();
            var s = fr.getSin();
            double x = m.distanceMeters * c;
            double y = m.distanceMeters * s;


            // select the compensation based on the position of the module wheel location
            Pose2d modulePose = robotPose.plus(new Transform2d(modules[i], new Rotation2d()));
            var comp = modulePose.getY() < centerLine ? sourceSideFieldCompensation : ampSideFieldCompensation;

            // multiply compensation
            x *= c > 0 ? comp.posX : comp.negX;
            y *= s > 0 ? comp.posY : comp.negY;

            // recreate moduleDelta
            var dist = Math.sqrt(x * x + y * y);
            var angle = Math.atan2(y, x) - g.getRadians();
            moduleDeltas[i] = new SwerveModulePosition(dist, new Rotation2d(angle));
        }

        return super.toTwist2d(moduleDeltas);
    }
}
