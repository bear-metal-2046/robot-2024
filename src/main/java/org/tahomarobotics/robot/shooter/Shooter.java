package org.tahomarobotics.robot.shooter;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;

public class Shooter extends SubsystemIF {

    private static final Shooter INSTANCE = new Shooter();

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // CONSTRUCTOR

    private Shooter() {
        io = switch (RobotConfiguration.getMode()) {
            case REAL -> new ShooterIO();
            case SIM, REPLAY -> new ShooterIOSim();
        };
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        io.zero();

        return this;
    }

    // SETTERS

    public void disable() {
        io.disable();
    }

    public void toggleShootMode() {
        io.toggleShootMode();
    }

    public void setAngle(double angle) {
        inputs.angle = MathUtil.clamp(angle, MIN_PIVOT_ANGLE, MAX_PIVOT_ANGLE);
    }

    public void biasUp() {
        setAngle(inputs.angle + BIAS_AMT);
    }

    public void biasDown() {
        setAngle(inputs.angle - BIAS_AMT);
    }

    // GETTERS

    private double getShooterVelocity() {
        return io.getShooterVelocity();
    }

    private double getPivotPosition() {
        return io.getPivotPosition();
    }

    public boolean isReadyToShoot() {
        return io.isReadyToShoot();
    }

    public boolean inShootingMode() {
        return io.inShootingMode();
    }

    public double rotToSpeaker() {
        return io.rotToSpeaker();
    }

    public double angleToSpeaker() {
        return io.angleToSpeaker();
    }

    // PERIODIC

    @Override
    public void periodic() {
        io.processInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/Velocity", getShooterVelocity());
        Logger.recordOutput("Shooter/Angle", getPivotPosition());
        Logger.recordOutput("Shooter/Angle (Degrees)", getPivotPosition() * 360);


    }

    // onInit

    @Override
    public void onDisabledInit() {
        disable();
    }
}
