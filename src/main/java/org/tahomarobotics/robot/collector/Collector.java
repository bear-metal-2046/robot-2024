package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class Collector extends SubsystemIF {

    private static final Collector INSTANCE = new Collector();

    public static Collector getInstance() {
        return INSTANCE;
    }

    private final TalonFX deployMotor;
    private final TalonFX deployFollower;
    private final TalonFX collectMotor;

    private final StatusSignal<Double> deployPosition;
    private final StatusSignal<Double> deployVelocity;
    private final StatusSignal<Double> collectVelocity;

    private final MotionMagicVelocityVoltage collectVelocityControl = new MotionMagicVelocityVoltage(COLLECT_MAX_RPS).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);
    private final MotionMagicVoltage deployPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(RobotConfiguration.USING_PHOENIX_PRO);
    private final VoltageOut voltage = new VoltageOut(0.0);

    private final SysIdRoutine routine;

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_rotations = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    private final RobustConfigurator configurator;

    public Collector() {
        configurator = new RobustConfigurator(logger);

        deployMotor = new TalonFX(RobotMap.DEPLOY_COLLECTOR_MOTOR);
        deployFollower = new TalonFX(RobotMap.DEPLOY_COLLECTOR_MOTOR_FOLLOWER);
        collectMotor = new TalonFX(RobotMap.COLLECTOR_MOTOR);

        configurator.configureTalonFX(deployMotor, deployMotorConfiguration, deployFollower, true);
        configurator.configureTalonFX(collectMotor, collectMotorConfiguration);

        deployPosition = deployMotor.getPosition();
        deployVelocity = deployMotor.getVelocity();
        collectVelocity = collectMotor.getVelocity();

        ParentDevice.optimizeBusUtilizationForAll(deployMotor, deployFollower, collectMotor);

        routine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (Measure<Voltage> volts) -> {
                            deployMotor.setControl(voltage.withOutput(volts.in(Volts)));
                        },
                        // Tell SysId how to record a frame of data for each motor on the mechanism being
                        // characterized.
                        log -> {
                            // Record a frame for the left motors.  Since these share an encoder, we consider
                            // the entire group to be one motor.
                            log.motor("drive-left")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    deployMotor.getMotorVoltage().getValue() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_rotations.mut_replace(deployPosition.refresh().getValue(), Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(deployVelocity.refresh().getValue(), RotationsPerSecond));
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test state in
                        // WPILog with this subsystem's name ("drive")
                        this));

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(deployPosition, collectVelocity);
    }

    @Override
    public SubsystemIF initialize() {
        deployMotor.setPosition(0);
        return this;
    }

    public void zeroCollector() {
        deployMotor.setPosition(0);
    }

    public boolean isAtPosition(double desiredPosition) {
        return Math.abs(getDeployPosition() - desiredPosition) < EPSILON;
    }

    public double getDeployPosition() {
        return deployPosition.refresh().getValue() * DEPLOY_GEAR_REDUCTION;
    }

    public double getCollectVelocity() {
        return collectVelocity.refresh().getValue() * COLLECT_GEAR_REDUCTION;
    }

    public void setDeployPosition(double position) {
        deployMotor.setControl(deployPositionControl.withPosition(position));
    }

    public void collect() {
        collectMotor.setControl(collectVelocityControl);
    }

    public void stopCollect() {
        collectMotor.stopMotor();
    }

    public Command getDeployCommand(double desiredPosition) {
        return new DeployCommand(this, desiredPosition);
    }
}
