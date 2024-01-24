package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class SysIdTest extends SubsystemIF {
    private final TalonFX motor;
    private final VoltageOut control = new VoltageOut(0);

    private final SysIdRoutine sysIdRoutine;

    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> position = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    public SysIdTest(SubsystemIF subsystem, TalonFX testingMotor) {

        //I'm only like 90% sure you can do this...
        //Just make sure to configure motors before ig
        this.motor = testingMotor;

        /* Speed up signals for better characterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */
        motor.optimizeBusUtilization();

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,         // Default ramp rate is acceptable
                        Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                        null,          // Default timeout is acceptable
                        null),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> motor.setControl(control.withOutput(volts.in(Volts))),
                        log -> log.motor("motor")
                                .voltage(
                                        voltage.mut_replace(
                                                motor.getMotorVoltage().getValue(), Volts))
                                .angularPosition(position.mut_replace(motor.getPosition().getValue(), Rotations))
                                .angularVelocity(
                                        velocity.mut_replace(motor.getVelocity().getValue(), RotationsPerSecond)),
                        subsystem));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}