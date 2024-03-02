package org.tahomarobotics.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.identity.RobotIdentity;
import org.tahomarobotics.robot.shooter.commands.ZeroShooterCommand;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.EnumSet;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;

public class Shooter extends SubsystemIF {

    private static final Shooter INSTANCE = new Shooter();

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double biasAngle = 0.0;
    protected double distance = 0.0;
    private double energyUsed = 0;

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
        SmartDashboard.putBoolean("Debug/NoIdleVelocity", true);
        NetworkTableInstance.getDefault().addListener(SmartDashboard.getEntry("Debug/NoIdleVelocity"), EnumSet.of(NetworkTableEvent.Kind.kValueAll), e -> disable());

        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ZeroShooterCommand().andThen(Commands.runOnce(this::disable)))
                .ignoringDisable(true).schedule();

        io.zero();

        return this;
    }

    // SETTERS

    public void disable() {
        io.disableShooter();
    }

    public void enable() {
        io.enableShooter();
    }

    public void disableShootMode() {
        io.disableShootMode();
    }

    public void enableShootMode() {
        io.enableShootMode();
    }

    public void stop() {
        io.stop();
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

    public boolean isSpinningAtVelocity() {
        return io.isSpinningAtVelocity();
    }

    public boolean inShootingMode() {
        return io.inShootingMode();
    }

    public boolean isAtAngle() {
        return io.isAtAngle();
    }

    public double getPivotVelocity() {
        return io.getPivotVelocity();
    }

    public void angleToSpeaker(double radialVelocity) {

        Translation2d target = SPEAKER_TARGET_POSITION.get();
        distance = Chassis.getInstance().getPose().getTranslation().getDistance(target) + SHOOTER_PIVOT_OFFSET.getX();

        SafeAKitLogger.recordOutput("Shooter/Radial Velocity", radialVelocity);
        SafeAKitLogger.recordOutput("Shooter/Target Angle Before Compensation", 0.07068257 + 0.1999213 * Math.pow(Math.E, -0.5485811 * distance));
        distance = (radialVelocity *
                (radialVelocity > 0 ? TIME_SHOT_OFFSET_POSITIVE : TIME_SHOT_OFFSET_NEGATIVE))
                + distance;

        setAngle(switch (RobotIdentity.robotID) {
            // y = 0.07068257 + 0.1999213*e^(-0.5485811*x)
            case PLAYBEAR_CARTI -> 0.07068257 + 0.1999213 * Math.pow(Math.E, -0.5485811 * distance);
            // y = 0.0000369x^4 - 0.00108x^3 + 0.0126x^2 - 0.0706x + 0.234
            case BEARITONE -> 0.0000369 * Math.pow(distance, 4) - 0.00108 * Math.pow(distance, 3) + 0.0126 * Math.pow(distance, 2) - 0.0706 * distance + 0.234;
            default -> 0.04875446 + (0.201136 - 0.04875446)/(1 + Math.pow((distance/2.019404), 2.137465)) + 0.002;
        });
    }

    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }

    public void zero() {
        io.zero();
    }

    // PERIODIC

    @Override
    public void periodic() {
        io.refreshSignals();
        io.processInputs(inputs);
        double voltage = RobotController.getBatteryVoltage();
        energyUsed += io.getTotalCurrent() * voltage * Robot.defaultPeriodSecs;

        Logger.processInputs("Shooter", inputs);

        SafeAKitLogger.recordOutput("Shooter/Bias", biasAngle);
        SafeAKitLogger.recordOutput("Shooter/Distance To Speaker", distance);
        SafeAKitLogger.recordOutput("Shooter/Is at Angle", isAtAngle());
        SafeAKitLogger.recordOutput("Shooter/Is Spinning At velocity", isSpinningAtVelocity());
        SafeAKitLogger.recordOutput("Shooter/Is In Shooting Mode", inShootingMode());
        SafeAKitLogger.recordOutput("Shooter/Distance", distance);
        SafeAKitLogger.recordOutput("Shooter/Velocity", getShooterVelocity());
        SafeAKitLogger.recordOutput("Shooter/Angle", getPivotPosition());
        SafeAKitLogger.recordOutput("Shooter/Angle (Degrees)", getPivotPosition() * 360);

        SafeAKitLogger.recordOutput("Shooter/TotalCurrent", io.getTotalCurrent());
        SafeAKitLogger.recordOutput("Shooter/Energy", getEnergyUsed());


    }

    // onInit

    @Override
    public void onDisabledInit() {
        stop();
    }

    @Override
    public double getEnergyUsed() {
        return energyUsed / 1000d;
    }
}
