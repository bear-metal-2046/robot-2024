package org.tahomarobotics.robot.shooter;

public class ShooterIOSim extends ShooterIO {
    @Override
    public double getPivotPosition() {
        return angle;
    }

    @Override
    public double getShooterVelocity() {
        return ShooterConstants.SHOOTER_SPEED;
    }
}
