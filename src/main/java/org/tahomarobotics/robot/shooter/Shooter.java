package org.tahomarobotics.robot.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

    public void enableReadyMode() {
        io.enableReadyMode();
    }
    public void disableReadyMode() {
        io.disableReadyMode();
    }
    public void toggleReadyMode() {
        io.toggleReadyMode();
    }
    public void toggleRedundantShootMode(){
        io.toggleRedundantShootMode();
    }

    public void toggleIdle() {
        io.toggleIdle();
    }

    public void setAngle(double angle) {
        inputs.angle = MathUtil.clamp(angle + (io.inReadyMode() ? biasAngle : 0), MIN_PIVOT_ANGLE, MAX_PIVOT_ANGLE);
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

    public boolean inReadyMode() {
        return io.inReadyMode();
    }

    public ShootMode getShootMode() {
        return io.getShootMode();
    }

    public void enablePassHigh(){
        io.enablePassHigh();
    }
    public void enablePassLow() {
        io.enablePassLow();
    }

    public void togglePassLow() {
        io.togglePassLow();
    }

    public void togglePassHigh() {
        io.togglePassHigh();
    }

    public boolean inPassingMode() {
        return io.inPassingMode();
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

    public void angleToSpeaker() {
        if (RobotState.isAutonomous() && Autonomous.getInstance().isUsingLookupTable()) {
            return;
        }

        Translation2d target = SPEAKER_TARGET_POSITION.get();
        distance = Chassis.getInstance().getPose().getTranslation().getDistance(target) + SHOOTER_PIVOT_OFFSET.getX();

        SafeAKitLogger.recordOutput("Shooter/Target Angle Before Compensation", speakerAngleCalc(distance));

//        double timeShotOffset = (radialVelocity > 0 ? TIME_SHOT_OFFSET_POSITIVE : TIME_SHOT_OFFSET_NEGATIVE);
        double targetAngle = speakerAngleCalc(distance);

        setAngle(targetAngle * GEAR_REDUCTION_COMPENSATION);
    }

    public void angleToPass() {
        setAngle((getShootMode().equals(ShootMode.PASSING_LOW)) ? LOW_PASS_POS : HIGH_PASS_POS);
    }

    private double speakerAngleCalc(double distance) {
        return switch (RobotIdentity.robotID) {
            // y = .1823 * e ^ (-.5392 * x) + 0.05025
            case BEARITONE, PLAYBEAR_CARTI -> 0.1823 * Math.pow(Math.E, -0.5392 * distance) + 0.05025;
            default -> 0.04875446 + (0.201136 - 0.04875446)/(1 + Math.pow((distance/2.019404), 2.137465)) + 0.002;
        };
    }

    private double passAngleCalc(double distance) {
        return switch (RobotIdentity.robotID) {
            // y = .1823 * e ^ (-.5392 * x) + 0.15025
            case BEARITONE, PLAYBEAR_CARTI -> 0.1823 * Math.pow(Math.E, -0.5392 * distance) + 0.15025;
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
        SafeAKitLogger.recordOutput("Shooter/Is In Shooting Mode", inReadyMode());
        SafeAKitLogger.recordOutput("Shooter/Distance", distance);
        SafeAKitLogger.recordOutput("Shooter/Top Velocity", getTopShooterVelocity());
        SafeAKitLogger.recordOutput("Shooter/Top Voltage", io.getTopShooterVoltage());
        SafeAKitLogger.recordOutput("Shooter/Bottom Velocity", getBottomShooterVelocity());
        SafeAKitLogger.recordOutput("Shooter/Bottom Voltage", io.getBottomShooterVoltage());
        SafeAKitLogger.recordOutput("Shooter/Angle", getPivotPosition());
        SafeAKitLogger.recordOutput("Shooter/Angle (Degrees)", getPivotPosition() * 360);

        SafeAKitLogger.recordOutput("MotorCurrents/Shooter Bottom", io.getTopShooterMotorCurrent());
        SafeAKitLogger.recordOutput("MotorCurrents/Shooter Top", io.getBottomShooterMotorCurrent());
        SafeAKitLogger.recordOutput("MotorCurrents/Shooter Pivot", io.getPivotMotorCurrent());

        SafeAKitLogger.recordOutput("Shooter/TotalCurrent", totalCurrent);
        SafeAKitLogger.recordOutput("Shooter/Energy", getEnergyUsed());
        SafeAKitLogger.recordOutput("Shooter/ShootMode", io.getShootMode());
        SmartDashboard.putBoolean("Shooter/IdleMode", io.isIdling());
        SmartDashboard.putNumber("Shooter/Bias", biasAngle);

        io.periodic();
    }

    // onInit

    @Override
    public void onDisabledInit() {
        stop();
    }

    @Override
    public void onTeleopInit() {
        disableReadyMode();
        io.configureShooterForTeleop();
    }

    @Override
    public double getEnergyUsed() {
        return energyUsed / 1000d;
    }

    @Override
    public double getTotalCurrent() {
        return totalCurrent;
    }

    public Double getPivotCurrent() {
        return io.getPivotMotorCurrent();
    }

    public Double getTopShooterCurrent() {
        return io.getTopShooterCurrent();
    }

    public Double getBottomShooterCurrent() {
        return io.getBottomShooterCurrent();
    }

    public enum ShootMode {
        SHOOTING,
        PASSING_HIGH,
        PASSING_LOW
    }
}
