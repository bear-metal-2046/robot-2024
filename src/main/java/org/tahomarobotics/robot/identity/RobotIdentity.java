package org.tahomarobotics.robot.identity;


import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.util.SafeAKitLogger;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

public class RobotIdentity extends SubsystemIF {
    private static final Logger logger = LoggerFactory.getLogger(RobotIdentity.class);

    public final static RobotID robotID;

    static {
        RobotID tmp = null;
        for (byte[] address : getRobotAddress()) {
            for (RobotID identity : RobotID.values()) {
                if (Arrays.equals(identity.getMac(), address)) {
                    tmp = identity;
                    break;
                }
            }
            if (tmp != null) break;
        }

        if (tmp == null) {
            tmp = RobotID.ALPHA;
            logger.error("Could not get a valid ID for robot, assigning default");
        }
        robotID = tmp;

        logger.info("Set robot identity to " + tmp);

        SafeAKitLogger.recordOutput("Identity/RobotID", robotID);
        SafeAKitLogger.recordOutput("Identity/MAC", macToString(robotID.getMac()));
    }

    private static List<byte[]> getRobotAddress() {
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

    private static String macToString(byte[] address) {
        StringBuilder stringBuilder = new StringBuilder();

        for (int i = 0; i < address.length; i++) {
            if (i != 0) {
                stringBuilder.append(":");
            }
            stringBuilder.append(String.format("%02X", address[i]));
        }
        return stringBuilder.toString();
    }

    private static String macToArray(byte[] address) {
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

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }
}
