package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.Watchdog;

public class FauxWatchdog extends Watchdog {
    public FauxWatchdog() {
        super(0, () -> {});
    }

    @Override
    public void addEpoch(String epochName) {}

    @Override
    public void printEpochs() {}

    @Override
    public void enable() {}

    @Override
    public void disable() {}
}
