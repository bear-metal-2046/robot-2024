package org.tahomarobotics.robot.identity;

public enum RobotID {
    //Build season alpha bot
    ALPHA {
        @Override
        public byte[] getMac() {
            return new byte[]{};
        }
    },

    CHASSISBOT {
        @Override
        public byte[] getMac() {
            return new byte[]{0x00, (byte) 0x80, 0x2F, 0x33, 0x04, (byte) 0xF9};
        }
    },

    //One of the two comp bots, rename accordingly later
    PRACTICE {
        @Override
        public byte[] getMac() {
            return new byte[]{};
        }
    },

    //Primary comp bot, rename accordingly later
    COMPETITION {
        @Override
        public byte[] getMac() {
            return new byte[]{};
        }
    };

    public abstract byte[] getMac();
}
