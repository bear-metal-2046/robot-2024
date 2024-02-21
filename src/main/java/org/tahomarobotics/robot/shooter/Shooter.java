package org.tahomarobotics.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.ToggledOutputs;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;

public class Shooter extends SubsystemIF implements ToggledOutputs {

    private static final Shooter INSTANCE = new Shooter();

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double biasAngle = 0.0;
    protected double distance = 0.0;
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
        inputs.angle = MathUtil.clamp(angle + (io.inShootingMode() ? biasAngle : 0), MIN_PIVOT_ANGLE, MAX_PIVOT_ANGLE);
    }

    public void biasUp() {
        biasAngle += BIAS_AMT;
    }

    public void biasDown() {
        biasAngle -= BIAS_AMT;
    }

    public void resetBias() {
        biasAngle = 0.0;
    }

    public void transferToAmp() {
        io.transferToAmp();
    }

    public void reverseIntake() {
        io.reverseIntake();
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

    public boolean isAtAngle() {
        return io.isAtAngle();
    }

    public void angleToSpeaker(double radialVelocity) {

        Translation2d target = SPEAKER_TARGET_POSITION.get();
        distance = Chassis.getInstance().getPose().getTranslation().getDistance(target) + SHOOTER_PIVOT_OFFSET.getX();

        recordOutput("Shooter/Target Angle Before Compensation", 0.04875446 + (0.201136 - 0.04875446)/(1 + Math.pow((distance/2.019404), 2.137465)));

        distance = (radialVelocity * TIME_SHOT_OFFSET) + distance;

        setAngle(0.04875446 + (0.201136 - 0.04875446)/(1 + Math.pow((distance/2.019404), 2.137465)) + 0.002);
    }

    // PERIODIC

    @Override
    public void periodic() {
        io.refreshSignals();
        io.processInputs(inputs);

        Logger.processInputs("Shooter", inputs);

        recordOutput("Shooter/Bias", biasAngle);
        recordOutput("Shooter/Distance", distance);
        recordOutput("Shooter/Velocity", getShooterVelocity());
        recordOutput("Shooter/Angle", getPivotPosition());
        recordOutput("Shooter/Angle (Degrees)", getPivotPosition() * 360);
    }

    // onInit

    @Override
    public void onDisabledInit() {
        disable();
    }

    @Override
    public boolean logOutputs() {
        return OutputsConfiguration.SHOOTER;
    }
}
