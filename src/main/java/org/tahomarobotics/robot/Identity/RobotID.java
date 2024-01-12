package org.tahomarobotics.robot.Identity;

public enum RobotID {
    ALPHA {
        @Override
        public byte[] getMac() {
            return new byte[]{};
        }
    };

    public abstract byte[] getMac();
}
