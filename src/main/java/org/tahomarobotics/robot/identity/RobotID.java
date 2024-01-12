package org.tahomarobotics.robot.identity;

public enum RobotID {
    //Build season alpha bot
    ALPHA {
        @Override
        public byte[] getMac() {return new byte[]{};}
    },

    //One of the two comp bots, rename accordingly later
    PRACTICE {
        @Override
        public byte[] getMac() {return new byte[]{};}
    },

    //Primary comp bot, rename accordingly later
    COMPETITION {
        @Override
        public byte[] getMac() {return new byte[]{};}
    };

    public abstract byte[] getMac();
}
