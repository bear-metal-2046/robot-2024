package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.OutputsConfiguration;
import org.tahomarobotics.robot.util.ToggledOutputs;

import java.util.EnumSet;

public class ObjectDetectionCamera implements ToggledOutputs {
    private static final Logger logger = LoggerFactory.getLogger(ObjectDetectionCamera.class);

    private final PhotonCamera camera;
    private final VisionConstants.Camera cameraSettings;
    private Translation3d notePosition = new Translation3d();

    public ObjectDetectionCamera(VisionConstants.Camera cameraSettings) {
        this.cameraSettings = cameraSettings;

        SmartDashboard.putBoolean("Outputs/ObjectDetection", OutputsConfiguration.OBJECT_DETECTION);

        // normally this would the default client connecting to robot
        // connect to server running on camera (for debugging0
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // create PhotonLib camera (required for each camera)
        camera = new PhotonCamera(inst, cameraSettings.cameraName);

        // subscribe to new data from photon vision
        DoubleSubscriber latencySub = inst.getTable("photonvision").getSubTable(cameraSettings.cameraName).getDoubleTopic("latencyMillis").subscribe(0.0);
        inst.addListener(
                latencySub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> processVisionUpdate(camera.getLatestResult())
        );
    }

    private void processVisionUpdate(PhotonPipelineResult result) {
        if (!result.hasTargets()) return;

        var target = result.getBestTarget();
        double xOffset = PhotonUtils.calculateDistanceToTargetMeters(
                cameraSettings.offset.getZ(),
                0,
                cameraSettings.offset.getRotation().getY(),
                Units.degreesToRadians(target.getPitch())
        ) + cameraSettings.offset.getX();

        double yOffset = (Math.tan(Units.degreesToRadians(-target.getYaw())) * xOffset) + cameraSettings.offset.getY();

        notePosition = new Translation3d(xOffset, yOffset, 0);
        recordOutput("Object Detection/Note Position", notePosition);
    }

    public Translation3d getNotePosition() {
        return notePosition;
    }

    @Override
    public boolean logOutputs() {
        return SmartDashboard.getBoolean("Outputs/ObjectDetection", OutputsConfiguration.OBJECT_DETECTION);
    }
}
