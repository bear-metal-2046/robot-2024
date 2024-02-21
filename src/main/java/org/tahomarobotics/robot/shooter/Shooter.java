package org.tahomarobotics.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.identity.RobotIdentity;
import org.tahomarobotics.robot.shooter.commands.ZeroShooterCommand;
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
        SmartDashboard.putBoolean("Outputs/Shooter", OutputsConfiguration.SHOOTER);

        Commands.waitUntil(RobotState::isEnabled)
                .andThen(new ZeroShooterCommand())
                .ignoringDisable(true).schedule();

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

    public double getPivotVelocity() {
        return io.getPivotVelocity();
    }

    public void angleToSpeaker(double radialVelocity) {

        Translation2d target = SPEAKER_TARGET_POSITION.get();
        distance = Chassis.getInstance().getPose().getTranslation().getDistance(target) + SHOOTER_PIVOT_OFFSET.getX();

        recordOutput("Shooter/Target Angle Before Compensation", 0.04875446 + (0.201136 - 0.04875446)/(1 + Math.pow((distance/2.019404), 2.137465)));

        distance = (radialVelocity * TIME_SHOT_OFFSET) + distance;

        setAngle(switch (RobotIdentity.getInstance().getRobotID()) {
            // y = 0.07068257 + 0.1999213*e^(-0.5485811*x)
            case PLAYBEAR_CARTI, BEARITONE -> 0.07068257 + 0.1999213 * Math.pow(Math.E, -0.5485811 * distance);
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
        return SmartDashboard.getBoolean("Outputs/Shooter", OutputsConfiguration.SHOOTER);
    }
}
