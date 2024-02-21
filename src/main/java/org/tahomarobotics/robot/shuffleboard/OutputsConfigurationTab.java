package org.tahomarobotics.robot.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.EnumSet;
import java.util.Map;

import static org.tahomarobotics.robot.OutputsConfiguration.*;

public class OutputsConfigurationTab {
    public static void create() {
        ShuffleboardTab tab = Shuffleboard.getTab("Outputs");
        ShuffleboardLayout layout = tab.getLayout("Outputs", BuiltInLayouts.kGrid)
                .withSize(6, 2)
                .withPosition(0, 0);

        Map<String, Boolean> defaults = Map.of(
                "Chassis", CHASSIS,
                "SwerveModules", SWERVE_MODULE,
                "Collector", COLLECTOR,
                "Indexer", INDEXER,
                "Shooter", SHOOTER,
                "AmpArm", AMP_ARM,
                "ATVision", AT_VISION
        );

        defaults.forEach((name, value) -> {
            GenericEntry entry = layout
                    .add(name, value)
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .withSize(2, 1).getEntry();

            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            inst.addListener(
                    entry,
                    EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    v ->
                            SmartDashboard.putBoolean("Outputs/" + name, entry.getBoolean(false))
                    );
        });
    }
}
