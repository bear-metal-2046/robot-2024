package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;

public class SysIdTest extends SubsystemIF {
    private final TalonFX motorToTest;
    private final TalonFX follower;
    private final DutyCycleOut joystickControl = new DutyCycleOut(0);
    private final VoltageOut sysidControl = new VoltageOut(0);

    private final SysIdRoutine sysIdRoutine;
    private RobustConfigurator configurator = new RobustConfigurator(logger);

    public SysIdTest(TalonFX testingMotor, TalonFX follower) {

        //I'm only like 90% sure you can do this...
        //Just make sure to configure motors before ig
        this.motorToTest = testingMotor;
        this.follower = follower;

        /* Speed up signals for better characterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                motorToTest.getPosition(),
                motorToTest.getVelocity(),
                motorToTest.getMotorVoltage());

        /* Optimize out the other signals, since they're not particularly helpful for us */
        motorToTest.optimizeBusUtilization();

        SignalLogger.start();

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,         // Default ramp rate is acceptable
                        Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                        null,          // Default timeout is acceptable
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> motorToTest.setControl(sysidControl.withOutput(volts.in(Volts))),
                        null,
                        this));
    }

    public Command joystickCommand(DoubleSupplier output) {
        return run(() -> motorToTest.setControl(joystickControl.withOutput(output.getAsDouble())));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
