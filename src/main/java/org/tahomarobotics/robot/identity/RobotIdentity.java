package org.tahomarobotics.robot.identity;


import org.littletonrobotics.junction.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

public class RobotIdentity extends SubsystemIF {

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
            robotID = RobotID.BEARITONE;
            logger.error("Could not get a valid ID for robot, assigning default");
        }
        logger.info("Set robot identity to " + robotID);

        Logger.recordOutput("Identity/RobotID", robotID);
        Logger.recordOutput("Identity/MAC", macToString(robotID.getMac()));
    }

    private List<byte[]> getRobotAddress() {

        List<byte[]> addresses = new ArrayList<>();

        try {
            Enumeration<NetworkInterface> networkInterfaceEnumeration = NetworkInterface.getNetworkInterfaces();

            while (networkInterfaceEnumeration.hasMoreElements()) {
                NetworkInterface inter = networkInterfaceEnumeration.nextElement();
                String name = inter.getName();
                byte[] addr = inter.getHardwareAddress();
                if (addr == null || !name.startsWith("eth")) {
                    continue;
                }

                addresses.add(addr);

                logger.info("Interface: " + name + ", MAC address: " + macToString(addr) + ", Code: " + macToArray(addr));
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

    private String macToArray(byte[] address) {

        StringBuilder stringBuilder = new StringBuilder("[");

        for (int i = 0; i < address.length; i++) {
            if (i != 0) {
                stringBuilder.append(", ");
            }
            stringBuilder.append(String.format("0x%02X", address[i]));
        }
        stringBuilder.append("]");

        return stringBuilder.toString();
    }

    public RobotID getRobotID() {
        return robotID;
    }
}
