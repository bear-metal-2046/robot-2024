package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.tahomarobotics.robot.util.SafeAKitLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class ATVision {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    private final VisionConstants.Camera cameraSettings;
    private final Field2d fieldPose;
    private final SwerveDrivePoseEstimator poseEstimator;
    private int updates = 0;

    public ATVision(VisionConstants.Camera cameraSettings, Field2d fieldPose, SwerveDrivePoseEstimator poseEstimator) {
        this.cameraSettings = cameraSettings;
        this.fieldPose = fieldPose;
        this.poseEstimator = poseEstimator;

        // normally this would the default client connecting to robot
        // connect to server running on camera (for debugging0
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // create PhotonLib camera (required for each camera)
        camera = new PhotonCamera(inst, cameraSettings.cameraName);

        fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
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
                List.of(target.getFiducialId()),
                target.getPoseAmbiguity(),
                -1
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

        var multiRes = result.getMultiTagResult().estimatedPose;
        SafeAKitLogger.recordOutput("ATCamera/Reprojection Error", multiRes.bestReprojErr);
        if (multiRes.isPresent && multiRes.bestReprojErr < 15.0) {
            Pose3d pose = new Pose3d(multiRes.best.getTranslation(), multiRes.best.getRotation()).plus(cameraSettings.offset.inverse());

            double distances = 0;
            for (PhotonTrackedTarget target : validTargets) {
                Transform3d _pose = target.getBestCameraToTarget();
                distances = Math.max(distances, Math.hypot(_pose.getX(), _pose.getY()));
            }

            addVisionMeasurement(new ATCameraResult(
                    cameraSettings,
                    result.getTimestampSeconds(), // Photon vision timestamp
                    pose.toPose2d(),
                    distances,
                    result.getMultiTagResult().fiducialIDsUsed,
                    multiRes.ambiguity,
                    multiRes.bestReprojErr
            ));
        } else if (validTargets.size() == 1) {
            processSingleTarget(validTargets.get(0), result.getTimestampSeconds());
        }
    }

    private void addVisionMeasurement(ATCameraResult result) {
        Pose2d pose;

        synchronized (poseEstimator) {
            pose = poseEstimator.getEstimatedPosition();
        }

        double distanceToTargets = result.distanceToTargets();

        synchronized (fieldPose) {
            fieldPose.getObject(result.camera().cameraName).setPose(result.poseMeters());
        }

        double distanceToMidlineTrust = ((16.5417 / 2) - Math.abs((16.5417 / 2) - pose.getX()) * 2.0) / (16.5417 / 2);
        distanceToMidlineTrust *= 2.5;
        distanceToMidlineTrust += .5;
        distanceToMidlineTrust = Math.max(1, distanceToMidlineTrust);

        SafeAKitLogger.recordOutput("ATCamera/TRUST", distanceToMidlineTrust);

        SafeAKitLogger.recordOutput("ATCamera/" + cameraSettings.cameraName + "/Pose", result.poseMeters());
        SafeAKitLogger.recordOutput("ATCamera/" + cameraSettings.cameraName + "/Multitag?", result.targets().size() > 1);
        SafeAKitLogger.recordOutput("ATCamera/" + cameraSettings.cameraName + "/Number of Targets", result.targets().size());
        SafeAKitLogger.recordOutput("ATCamera/" + cameraSettings.cameraName + "/Ambiguity", result.ambiguity());
        SafeAKitLogger.recordOutput("ATCamera/" + cameraSettings.cameraName + "/Reprojection Error", result.reprojectionError());
        SafeAKitLogger.recordOutput("ATCamera/" + cameraSettings.cameraName + "/Updates", updates);
        SafeAKitLogger.recordOutput("ATCamera/" + cameraSettings.cameraName + "/Apriltag IDs", result.targets().toString());

        if (result.targets().size() > 1 && distanceToTargets < VisionConstants.TARGET_DISTANCE_THRESHOLD) {
            // Multi-tag PnP provides very trustworthy data
            var stds = VecBuilder.fill(
                    0.08122476428 * distanceToMidlineTrust,
                    0.0990676807 * distanceToMidlineTrust,
                    Units.degreesToRadians(1.372694632) * distanceToMidlineTrust * distanceToMidlineTrust
            );

            synchronized (poseEstimator) {
                poseEstimator.addVisionMeasurement(result.poseMeters(), result.timestamp(), stds);
                updates++;
            }
        } else if (result.targets().size() == 1 && distanceToTargets < VisionConstants.SINGLE_TARGET_DISTANCE_THRESHOLD) {
            // Single tag results are not very trustworthy. Do not use headings from them
            Pose2d noHdgPose = new Pose2d(result.poseMeters().getTranslation(), pose.getRotation());
            var stds = VecBuilder.fill(
                    0.25 * distanceToTargets,
                    0.25 * distanceToTargets,
                    Units.degreesToRadians(90)
            );

            synchronized (poseEstimator) {
                poseEstimator.addVisionMeasurement(noHdgPose, result.timestamp(), stds);
                updates++;
            }
        }
    }

    public void update() {
        processVisionUpdate(camera.getLatestResult());
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

    public record ATCameraResult(VisionConstants.Camera camera, double timestamp, Pose2d poseMeters,
                                 double distanceToTargets, List<Integer> targets, double ambiguity, double reprojectionError) {
    }
}