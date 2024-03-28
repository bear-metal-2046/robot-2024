package org.tahomarobotics.robot.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SafeAKitLogger;

import static org.tahomarobotics.robot.shooter.ShooterConstants.*;


class ShooterIO {
    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(ShooterIO.class);

    private final TalonFX topShooterMotor;
    private final TalonFX bottomShooterMotor;
    //private final TalonFX pivotMotor;

    private final StatusSignal<Double> topShooterVelocity;
    private final StatusSignal<Double> bottomShooterVelocity;
    //private final StatusSignal<Double> pivotPosition;
    //private final StatusSignal<Double> pivotVelocity;
    private final StatusSignal<Double> topShooterVoltage, bottomShooterVoltage;

    private final StatusSignal<Double> topShooterCurrent;
    private final StatusSignal<Double> bottomShooterCurrent;

    //private final StatusSignal<Double> pivotCurrent;

    private final MotionMagicVoltage pivotPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage motorVelocity = new MotionMagicVelocityVoltage(SHOOTER_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage idleVelocity = new MotionMagicVelocityVoltage(IDLE_SPEED).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage transferVelocity = new MotionMagicVelocityVoltage(TRANSFER_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVelocityVoltage reverseIntakeVelocity = new MotionMagicVelocityVoltage(-REVERSE_INTAKE_VELOCITY).withEnableFOC(RobotConfiguration.RIO_PHOENIX_PRO);

    protected double angle = 0.0;

    private boolean shootingMode = false;
    private boolean redundantShootingMode = false;
    private boolean idleMode = true;
    private boolean isZeroed = false;

    private double targetShooterSpeed = 0.0;

    @AutoLog
    static class ShooterIOInputs {

        double angle = 0.0;

    }
    ShooterIO() {
        RobustConfigurator configurator = new RobustConfigurator(logger);

        //pivotMotor = new TalonFX(RobotMap.SHOOTER_PIVOT_MOTOR);
        topShooterMotor = new TalonFX(RobotMap.TOP_SHOOTER_MOTOR);
        bottomShooterMotor = new TalonFX(RobotMap.BOTTOM_SHOOTER_MOTOR);

        //configurator.configureTalonFX(pivotMotor, pivotMotorConfiguration, "pivot motor");
        configurator.configureTalonFX(topShooterMotor, shooterMotorConfiguration, "shooter motor");
        configurator.configureTalonFX(bottomShooterMotor, shooterMotorConfiguration, "shooter motor follower");

        topShooterVelocity = topShooterMotor.getVelocity();
        bottomShooterVelocity = bottomShooterMotor.getVelocity();
        //pivotPosition = pivotMotor.getPosition();
        //pivotVelocity = pivotMotor.getVelocity();
        topShooterVoltage = topShooterMotor.getMotorVoltage();
        bottomShooterVoltage = bottomShooterMotor.getMotorVoltage();

        topShooterCurrent = topShooterMotor.getSupplyCurrent();
        bottomShooterCurrent = bottomShooterMotor.getSupplyCurrent();
        //pivotCurrent = pivotMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
                topShooterVelocity, bottomShooterVelocity, /*pivotVelocity,*/ topShooterVoltage,
                topShooterCurrent, bottomShooterCurrent, /*pivotCurrent,*/ bottomShooterVoltage
        );
        //ParentDevice.optimizeBusUtilizationForAll(pivotMotor, bottomShooterMotor, topShooterMotor);
    }

    // GETTERS

    double getTopShooterVelocity() {
        return topShooterVelocity.getValue();
    }

    double getBottomShooterVelocity() {
        return bottomShooterVelocity.getValue();
    }

    double getTopShooterCurrent() {
        return topShooterCurrent.getValueAsDouble();
    }

    double getBottomShooterCurrent() {
        return bottomShooterCurrent.getValueAsDouble();
    }

    double getTopShooterVoltage() {
        return topShooterVoltage.getValueAsDouble();
    }

    double getBottomShooterVoltage() {
        return bottomShooterVoltage.getValueAsDouble();
    }

//    double getPivotPosition() {
//        return BaseStatusSignal.getLatencyCompensatedValue(pivotPosition, pivotVelocity);
//    }

    boolean isSpinningAtVelocity() {
        return Math.abs(targetShooterSpeed - getTopShooterVelocity()) < SHOOTER_SPEED_TOLERANCE
                && Math.abs(targetShooterSpeed - getBottomShooterVelocity()) < SHOOTER_SPEED_TOLERANCE;
    }

//    boolean isAtAngle() {
//        return Math.abs(angle - getPivotPosition()) < PIVOT_ANGLE_TOLERANCE;
//    }

    boolean isReadyToShoot() {
        return /*isAtAngle() &&*/ isSpinningAtVelocity();
    }

    boolean inRedundantShootingMode(){return redundantShootingMode;}

    boolean inShootingMode() {
        return shootingMode;
    }

//    double getPivotVelocity() {
//        return pivotVelocity.getValue();
//    }

    boolean isIdling() {
        return idleMode;
    }

    // SETTERS

//    void setShooterAngle(double angle) {
//        this.angle = angle;
//
//        SafeAKitLogger.recordOutput("Shooter/Target Angle", angle);
//
//        pivotMotor.setControl(pivotPositionControl.withPosition(angle));
//    }

//    void zero() {
//        RobustConfigurator.retryConfigurator(() -> pivotMotor.setPosition(0.0),
//                "Zeroed Pivot Motor",
//                "FAILED TO SET PIVOT POSITION",
//                "Retrying setting pivot position.");
//
//        isZeroed = true;
//    }

    public boolean isZeroed() {
        return isZeroed;
    }

    void transferToAmp() {
        targetShooterSpeed = transferVelocity.Velocity;

        topShooterMotor.setControl(transferVelocity);
        bottomShooterMotor.setControl(transferVelocity);
    }

    void reverseIntake() {
        targetShooterSpeed = reverseIntakeVelocity.Velocity;

        topShooterMotor.setControl(reverseIntakeVelocity);
        bottomShooterMotor.setControl(reverseIntakeVelocity);
    }

//    void setPivotVoltage(double voltage) {
//        pivotMotor.setControl(new VoltageOut(voltage));
//    }


    // STATES
    void enable() {
        targetShooterSpeed = motorVelocity.Velocity;

        topShooterMotor.setControl(motorVelocity);
        bottomShooterMotor.setControl(motorVelocity);
    }

    void idle() {
        targetShooterSpeed = idleVelocity.Velocity;

        idleMode = true;
        topShooterMotor.setControl(idleVelocity);
        bottomShooterMotor.setControl(idleVelocity);
    }

    void stop() {
        targetShooterSpeed = 0.0;

        idleMode = false;
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }

    void toggleShootMode() {
        if (shootingMode) {
            disableShootMode();
            idle();
        } else if (Indexer.getInstance().isCollected()){
            enableShootMode();
            enable();
        }
    }

    void toggleRedundantShootMode(){
        if (redundantShootingMode){
            disableRedundantShootMode();
            idle();
        } else if (Indexer.getInstance().isCollected()) {
            enableRedundantShootMode();
            enable();
        }
    }

    public void toggleIdle() {
        if (idleMode) {
            stop();
        } else {
            idle();
        }
    }

    void enableShootMode() {
        shootingMode = true;
    }

    void disableShootMode() {
        shootingMode = false;
    }

    void enableRedundantShootMode(){
        redundantShootingMode = true;
    }

    void disableRedundantShootMode(){
        redundantShootingMode = false;
    }

    // INPUTS

    void processInputs(ShooterIOInputs inputs) {
//        setShooterAngle(inputs.angle);
    }

    void refreshSignals() {
        BaseStatusSignal.refreshAll(/*pivotPosition, pivotVelocity,*/ topShooterVelocity, bottomShooterVelocity,
                topShooterCurrent, bottomShooterCurrent, /*pivotCurrent,*/ topShooterVelocity, bottomShooterVoltage);
    }

    public double getTotalCurrent() {
        return Math.abs(topShooterCurrent.getValue()) + Math.abs(bottomShooterCurrent.getValue())/* + Math.abs(pivotCurrent.getValue())*/;
    }
}
