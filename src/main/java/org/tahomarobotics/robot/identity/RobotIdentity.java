package org.tahomarobotics.robot.identity;


import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

public class RobotIdentity {

    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(RobotIdentity.class);

    private static RobotID robotID;

    private static final RobotIdentity INSTANCE = new RobotIdentity();

    public static RobotIdentity getInstance() {
        return INSTANCE;
    }

    private RobotIdentity() {
        for (byte[] address : getRobotAddress()) {
            for (RobotID identity : RobotID.values()) {
                if (Arrays.equals(identity.getMac(), address)) {
                    robotID = identity;
                    break;
                }
            }
            if (robotID != null) break;
        }

        if (robotID == null) {
            robotID = RobotID.COMPETITION;
            logger.error("Could not get a valid ID for robot, assigning default");
        }
        logger.info("Set robot identity to " + robotID);

        Logger.recordOutput("RobotID", robotID);
    }

    private List<byte[]> getRobotAddress() {

        List<byte[]> addresses = new ArrayList<>();

        try {
            Enumeration<NetworkInterface> networkInterfaceEnumeration = NetworkInterface.getNetworkInterfaces();

            while (networkInterfaceEnumeration.hasMoreElements()) {
                byte[] address = networkInterfaceEnumeration.nextElement().getHardwareAddress();
                if (address == null) {
                    continue;
                }

                addresses.add(address);
                logger.info("MAC address: " + macToString(address));
                Logger.recordOutput("RobotID", macToString(address));
            }

        } catch (SocketException e) {
            logger.error("No MAC address found", e);
        }

        return addresses;
    }

    private String macToString(byte[] address) {

        StringBuilder stringBuilder = new StringBuilder();

        for (int i = 0; i < address.length; i++) {
            if (i != 0) {
                stringBuilder.append(":");
            }
            stringBuilder.append(String.format("%02X", address[i]));
        }
        return stringBuilder.toString();
    }

    public RobotID getRobotID() {
        return robotID;
    }
}
