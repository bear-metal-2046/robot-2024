package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

public class ATVision {
    private final Logger logger;
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final VisionConstants.ATCamera cameraSettings;
    private final Field2d fieldPose;
    private final SwerveDrivePoseEstimator poseEstimator;
    private int updates = 0;

    public ATVision(VisionConstants.ATCamera cameraSettings, Field2d fieldPose, SwerveDrivePoseEstimator poseEstimator) {
        logger = LoggerFactory.getLogger("ATVision." + cameraSettings.cameraName);
        this.cameraSettings = cameraSettings;
        this.fieldPose = fieldPose;
        this.poseEstimator = poseEstimator;

        // normally this would the default client connecting to robot
        // connect to server running on camera (for debugging0
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // create PhotonLib camera (required for each camera)
        camera = new PhotonCamera(inst, cameraSettings.cameraName);

        fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

        // Photon pose estimator will use all seen tags to run PnP once,
        // instead of looking at each target individually
        photonPoseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                cameraSettings.offset
        );

        // subscribe to new data from photon vision
        DoubleSubscriber latencySub = inst.getTable("photonvision").getSubTable(cameraSettings.cameraName).getDoubleTopic("latencyMillis").subscribe(0.0);
        inst.addListener(
                latencySub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                e -> processVisionUpdate(camera.getLatestResult())
        );
    }

    private void processSingleTarget(PhotonTrackedTarget target, double timestampSeconds) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
        Transform3d bestCamPose = target.getBestCameraToTarget();

        if (tagPose.isEmpty() || target.getPoseAmbiguity() > 0.2) {
            return;
        }

        double distance = Math.hypot(bestCamPose.getX(), bestCamPose.getY());
        Pose3d newRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                bestCamPose,
                tagPose.get(),
                cameraSettings.offset.inverse()
        );

        addVisionMeasurement(new ATCameraResult(
                cameraSettings,
                timestampSeconds, // Photon vision timestamp
                newRobotPose.toPose2d(),
                distance,
                1
        ));
    }

    /**
     * process new camera data from a subscribed camera
     *
     * @param result - new data from a camera
     */
    private void processVisionUpdate(PhotonPipelineResult result) {
        List<PhotonTrackedTarget> validTargets = new ArrayList<>();

        // Limelight can sometimes return tags that do not exist. Filter these out so that we have an accurate count.
        // This also fixes PhotonPoseEstimator using corners from bad results when doing PnP :(
        for (PhotonTrackedTarget target : result.targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                validTargets.add(target);
            }
        }

        if (validTargets.size() > 1 && photonPoseEstimator != null) {
            // Multi-tag PnP if more than one target is found
            PhotonPipelineResult correctedResult = new PhotonPipelineResult(result.getLatencyMillis(), validTargets);
            correctedResult.setTimestampSeconds(result.getTimestampSeconds());
            Optional<EstimatedRobotPose> position = photonPoseEstimator.update(correctedResult);

            if (position.isEmpty()) return;

            double distances = 0;
            for (PhotonTrackedTarget target : position.get().targetsUsed) {
                Transform3d pose = target.getBestCameraToTarget();
                distances = Math.max(distances, Math.hypot(pose.getX(), pose.getY()));
            }

            addVisionMeasurement(new ATCameraResult(
                    cameraSettings,
                    position.get().timestampSeconds, // Photon vision timestamp
                    position.get().estimatedPose.toPose2d(),
                    distances,
                    position.get().targetsUsed.size()
            ));
        } else if (validTargets.size() == 1) {
            processSingleTarget(validTargets.get(0), result.getTimestampSeconds());
        }
    }

    private void addVisionMeasurement(ATCameraResult result) {
        Pose2d pose;
        synchronized (poseEstimator) {
            pose = poseEstimator.getEstimatedPosition();

            Transform2d poseDiff = pose.minus(result.poseMeters());

            // Only add vision measurements close to where the robot currently thinks it is.
            if (poseDiff.getTranslation().getNorm() > VisionConstants.POSE_DIFFERENCE_THRESHOLD ||
                    poseDiff.getRotation().getDegrees() > VisionConstants.DEGREES_DIFFERENCE_THRESHOLD) {
                logger.warn("LARGE DISTANCE MOVED ACCORDING TO CAMERA '" + cameraSettings.cameraName + "', (" + poseDiff.getTranslation().getX() + "," + poseDiff.getTranslation().getY() + ")");
                return;
            }
        }

        double distanceToTargets = result.distanceToTargets();

        synchronized (fieldPose) {
            fieldPose.getObject(result.camera().cameraName).setPose(result.poseMeters());
        }

        if (result.numTargets() > 1 && distanceToTargets < VisionConstants.TARGET_DISTANCE_THRESHOLD) {
            // Multi-tag PnP provides very trustworthy data
            var stds = VecBuilder.fill(
                    0.01 * distanceToTargets,
                    0.01 * distanceToTargets,
                    0.01
            );

            synchronized (poseEstimator) {
                poseEstimator.addVisionMeasurement(result.poseMeters(), result.timestamp(), stds);
                updates++;
            }
        } else if (result.numTargets() == 1 && distanceToTargets < VisionConstants.SINGLE_TARGET_DISTANCE_THRESHOLD) {
            // Single tag results are not very trustworthy. Do not use headings from them
            Pose2d noHdgPose = new Pose2d(result.poseMeters().getTranslation(), pose.getRotation());
            var stds = VecBuilder.fill(
                    0.25 * distanceToTargets,
                    0.25 * distanceToTargets,
                    0.5
            );

            synchronized (poseEstimator) {
                poseEstimator.addVisionMeasurement(noHdgPose, result.timestamp(), stds);
                updates++;
            }
        }
    }

    public int getUpdates() {
        return updates;
    }

    public void resetUpdates() {
        updates = 0;
    }

    public String getName() {
        return cameraSettings.cameraName;
    }

    public record ATCameraResult(VisionConstants.ATCamera camera, double timestamp, Pose2d poseMeters,
                                 double distanceToTargets, int numTargets) {
    }
}