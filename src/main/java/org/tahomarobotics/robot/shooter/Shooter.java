package org.tahomarobotics.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.auto.Autonomous;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.identity.RobotIdentity;
import org.tahomarobotics.robot.shooter.commands.ZeroShooterCommand;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;

public class Shooter extends SubsystemIF {

    private static final Shooter INSTANCE = new Shooter();

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double biasAngle = 0.0;
    protected double distance = 0.0;
    private double energyUsed = 0;

    private double totalCurrent = 0;

    private boolean isShooting = false;

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
        Commands.waitUntil(() -> RobotState.isEnabled() && RobotState.isTeleop())
                .andThen(new ZeroShooterCommand())
                .ignoringDisable(true).schedule();

        io.zero();

        return this;
    }

    // SETTERS

    public void enable() {
        io.enable();
    }

    public void disableShootMode() {
        io.disableShootMode();
    }

    public void enableShootMode() {
        io.enableShootMode();
    }

    public void enableRedundantShootMode(){io.enableRedundantShootMode();}
    public void disableRedundantShootMode(){io.disableRedundantShootMode();}

    public void idle() {
        io.idle();
    }

    public void stop() {
        io.stop();
    }

    public void toggleShootMode() {
        io.toggleShootMode();
    }
    public void toggleRedundantShootMode(){
        io.toggleRedundantShootMode();
    }

    public void toggleIdle() {
        io.toggleIdle();
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

    public void setIsShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    // GETTERS

    private double getTopShooterVelocity() {
        return io.getTopShooterVelocity();
    }

    private double getBottomShooterVelocity() {
        return io.getBottomShooterVelocity();
    }

    private double getPivotPosition() {
        return io.getPivotPosition();
    }

    public boolean isReadyToShoot() {
        return io.isReadyToShoot();
    }

    public boolean isAtVelocity() {
        return io.isSpinningAtVelocity();
    }

    public boolean inShootingMode() {
        return io.inShootingMode();
    }

    public boolean inRedundantShootingMode() {
        return io.inRedundantShootingMode();
    }

    public boolean isAtAngle() {
        return io.isAtAngle();
    }

    public boolean isShooting() {
        return isShooting;
    }

    public double getPivotVelocity() {
        return io.getPivotVelocity();
    }

    public void angleToSpeaker(double radialVelocity) {
        if (RobotState.isAutonomous() && Autonomous.getInstance().isUsingLookupTable()) {
            return;
        }

        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue)
            radialVelocity *= -1;

        Translation2d target = SPEAKER_TARGET_POSITION.get();
        distance = Chassis.getInstance().getPose().getTranslation().getDistance(target) + SHOOTER_PIVOT_OFFSET.getX();

        SafeAKitLogger.recordOutput("Shooter/Target Angle Before Compensation", angleCalc(distance));

        double timeShotOffset = (radialVelocity > 0 ? TIME_SHOT_OFFSET_POSITIVE : TIME_SHOT_OFFSET_NEGATIVE);
        double targetAngle = angleCalc(distance + radialVelocity * timeShotOffset);

        setAngle(targetAngle);
    }

    private double angleCalc(double distance) {
        return switch (RobotIdentity.robotID) {
            // y = 0.07068257 + 0.1999213*e^(-0.5485811*x)
            case PLAYBEAR_CARTI -> 0.07068257 + 0.1999213 * Math.pow(Math.E, -0.5485811 * distance);
            // y = 0.00006435x^4 - 0.001775x^3 + 0.01868x^2 - 0.09331x + 0.2599
            case BEARITONE -> 0.00006435 * Math.pow(distance, 4) - 0.001775 * Math.pow(distance, 3) + 0.01868 * Math.pow(distance, 2) - 0.09331 * distance + 0.2599;
            default -> 0.04875446 + (0.201136 - 0.04875446)/(1 + Math.pow((distance/2.019404), 2.137465)) + 0.002;
        };
    }

    public void setPivotVoltage(double voltage) {
        io.setPivotVoltage(voltage);
    }

    public void zero() {
        io.zero();
    }

    public boolean isZeroed() {
        return io.isZeroed();
    }

    private void configureShooterForTeleop() {
        io.configureShooterForTeleop();
    }

    // PERIODIC

    @Override
    public void periodic() {
        io.refreshSignals();
        io.processInputs(inputs);
        double voltage = RobotController.getBatteryVoltage();
        totalCurrent = io.getTotalCurrent();
        energyUsed += totalCurrent * voltage * Robot.defaultPeriodSecs;

        Logger.processInputs("Shooter", inputs);

        SafeAKitLogger.recordOutput("Shooter/Bias Degrees", Units.rotationsToDegrees(biasAngle));
        SafeAKitLogger.recordOutput("Shooter/Distance To Speaker", distance);
        SafeAKitLogger.recordOutput("Shooter/Is at Angle", isAtAngle());
        SafeAKitLogger.recordOutput("Shooter/Is Spinning At velocity", isAtVelocity());
        SafeAKitLogger.recordOutput("Shooter/Is In Shooting Mode", inShootingMode());
        SafeAKitLogger.recordOutput("Shooter/Distance", distance);
        SafeAKitLogger.recordOutput("Shooter/Top Velocity", getTopShooterVelocity());
        SafeAKitLogger.recordOutput("Shooter/Top Current", io.getTopShooterCurrent());
        SafeAKitLogger.recordOutput("Shooter/Top Voltage", io.getTopShooterVoltage());
        SafeAKitLogger.recordOutput("Shooter/Bottom Velocity", getBottomShooterVelocity());
        SafeAKitLogger.recordOutput("Shooter/Bottom Current", io.getBottomShooterCurrent());
        SafeAKitLogger.recordOutput("Shooter/Bottom Voltage", io.getBottomShooterVoltage());
        SafeAKitLogger.recordOutput("Shooter/Angle", getPivotPosition());
        SafeAKitLogger.recordOutput("Shooter/Angle (Degrees)", getPivotPosition() * 360);

        SafeAKitLogger.recordOutput("Shooter/TotalCurrent", totalCurrent);
        SafeAKitLogger.recordOutput("Shooter/Energy", getEnergyUsed());
        SmartDashboard.putBoolean("Shooter/IdleMode", io.isIdling());
        SmartDashboard.putNumber("Shooter/Bias", biasAngle);


    }

    // onInit

    @Override
    public void onDisabledInit() {
        stop();
    }

    @Override
    public void onTeleopInit() {
        disableShootMode();
        configureShooterForTeleop();
    }

    @Override
    public double getEnergyUsed() {
        return energyUsed / 1000d;
    }

    @Override
    public double getTotalCurrent() {
        return totalCurrent;
    }
}
