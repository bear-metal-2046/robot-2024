package org.tahomarobotics.robot.identity;

public enum RobotID {
    ALPHA {
        @Override
        public byte[] getMac() {
            return new byte[]{0x00, (byte) 0x80, 0x2F, 0x32, (byte) 0xFC, (byte) 0xD7};
        }
    },

    PLAYBEAR_CARTI {
        @Override
        public byte[] getMac() {
            return new byte[]{0x00, (byte) 0x80, 0x2F, 0x33, 0x04, (byte) 0xF9};
        }
    },

    BEARITONE {
        @Override
        public byte[] getMac() {
            return new byte[]{0x00, (byte) 0x80, 0x2F, 0x32, (byte) 0xFD, 0x29};
        }
    };

    public abstract byte[] getMac();
}
