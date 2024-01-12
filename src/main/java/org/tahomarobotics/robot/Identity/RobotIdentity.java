package org.tahomarobotics.robot.Identity;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class RobotIdentity {

    private final Logger logger = LoggerFactory.getLogger(RobotIdentity.class);

    private static final RobotID robotID = null;

    private static final RobotIdentity INSTANCE = new RobotIdentity();

    public static RobotIdentity getInstance() {
        return INSTANCE;
    }

    private RobotIdentity() {

    }

    private List<byte[]> getRobotAddress() {

        List<byte[]> addresses = new ArrayList<>();

        try {
            Enumeration<NetworkInterface> networkInterfaceEnumeration = NetworkInterface.getNetworkInterfaces();

            NetworkInterface networkInterface;
            while (networkInterfaceEnumeration.hasMoreElements()) {
                byte[] address = networkInterfaceEnumeration.nextElement().getHardwareAddress();
                if (address == null) {
                    continue;
                }

                addresses.add(address);
                logger.info("MAC address: " + macToString(address));
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
