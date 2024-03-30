package org.tahomarobotics.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.tahomarobotics.robot.util.SafeAKitLogger;

import java.util.List;
import java.util.Optional;

public class ATVision {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    private final VisionConstants.Camera cameraSettings;
    private final Field2d fieldPose;
    private final SwerveDrivePoseEstimator poseEstimator;
    private int updates = 0;
    private double lastUpdateTime = 0;

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

    private ATCameraResult processSingleTarget(Pose2d robotPose, PhotonPipelineResult result) {
        Pose3d tagPose3d = null;
        PhotonTrackedTarget tagResult = null;

        // 1. Find valid target. There could be invalid tags that are not part of the field which we want to ignore.

        for (PhotonTrackedTarget target : result.targets) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                tagPose3d = tagPose.get();
                tagResult = target;
                break;
            }
        }

        if (tagPose3d == null) {
            return null;
        }

        // 2. Get robot pose from camera pose

        Transform3d camTransform0 = tagResult.getBestCameraToTarget();
        Transform3d camTransform1 = tagResult.getAlternateCameraToTarget();

        Pose3d camPose0 = PhotonUtils.estimateFieldToRobotAprilTag(
                camTransform0,
                tagPose3d,
                cameraSettings.offset.inverse()
        );

        Pose3d camPose1 = PhotonUtils.estimateFieldToRobotAprilTag(
                camTransform1,
                tagPose3d,
                cameraSettings.offset.inverse()
        );

        // 3. Pick the pose closest to the robot position/heading.
        //    This may help in higher ambiguity scenarios where the solution is not as clear.
        //    Based off of Mechanical Advantage's solution for this year:
        //    https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/264#filtering-11

        double camHdg0 = camPose0.getRotation().getZ();
        double camHdg1 = camPose1.getRotation().getZ();
        double robotHdg = robotPose.getRotation().getRadians();

        Pose3d bestPose, altPose;

        if (Math.abs(robotHdg - camHdg1) < Math.abs(robotHdg - camHdg0)) {
            bestPose = camPose1;
            altPose = camPose0;
        } else {
            bestPose = camPose0;
            altPose = camPose1;
        }

        // 4. Get distance and return result

        double distance = Math.hypot(bestPose.getX(), bestPose.getY());

        return new ATCameraResult(
                VecBuilder.fill(
                    0.25 * distance,
                    0.25 * distance,
                    Units.degreesToRadians(90)
                ),
                new ATCameraResult.ATCameraResultPose(bestPose, 0),
                new ATCameraResult.ATCameraResultPose(altPose, 0),
                tagResult.getPoseAmbiguity(),
                distance,
                List.of(tagResult.getFiducialId())
        );
    }

    private ATCameraResult processMultiTarget(Pose2d robotPose, MultiTargetPNPResult result) {

        // 1. Get Average distance between camera and tags

        double distance = 0;

        for (var tagId : result.fiducialIDsUsed) {
            Optional<Pose3d> tagPose3d = fieldLayout.getTagPose(tagId);

            // Tag pose should always exist as Photonvision already checks that tags are valid
            if (tagPose3d.isEmpty()) continue;

            Pose2d tagPose2d = tagPose3d.get().toPose2d();
            distance += tagPose2d.relativeTo(robotPose).getTranslation().getNorm();
        }

        distance /= result.fiducialIDsUsed.size();

        // 2. Get robot pose from camera pose

        Transform3d bestTransform = result.estimatedPose.best;
        Transform3d altTransform = result.estimatedPose.alt;

        Pose3d bestPose = new Pose3d(
                bestTransform.getTranslation(),
                bestTransform.getRotation()
            ).plus(cameraSettings.offset.inverse());

        Pose3d altPose = new Pose3d(
                altTransform.getTranslation(),
                altTransform.getRotation()
            ).plus(cameraSettings.offset.inverse());

        // 3. Calculate trust in new position
        //    Trust the robot pose the closer to centerline it is

        double midline = VisionConstants.FIELD_LENGTH / 2;
        double distanceToMidlineTrust = (midline - Math.abs(midline - robotPose.getX()) * 2.0) / midline;
        distanceToMidlineTrust *= 2.5;
        distanceToMidlineTrust += .5;
        distanceToMidlineTrust = Math.max(1, distanceToMidlineTrust);

        SafeAKitLogger.recordOutput("ATCamera/TRUST", distanceToMidlineTrust);

        // 3. Create result

        return new ATCameraResult(
                VecBuilder.fill(
                    0.08122476428 * distanceToMidlineTrust,
                    0.0990676807 * distanceToMidlineTrust,
                    Units.degreesToRadians(5.0) * distanceToMidlineTrust
                ),
                new ATCameraResult.ATCameraResultPose(bestPose, result.estimatedPose.bestReprojErr),
                new ATCameraResult.ATCameraResultPose(altPose, result.estimatedPose.altReprojErr),
                result.estimatedPose.ambiguity,
                distance,
                result.fiducialIDsUsed
        );
    }

    /**
     * process new camera data from a subscribed camera
     *
     * @param result - new data from a camera
     */
    private void processVisionUpdate(PhotonPipelineResult result) {
        Pose2d robotPose;
        String prefix = "ATCamera/" + cameraSettings.cameraName;
        MultiTargetPNPResult multiRes = result.getMultiTagResult();

        // Check for duplicate frame
        if (result.getTimestampSeconds() == lastUpdateTime) {
            SafeAKitLogger.recordOutput(prefix + "/New Frame", false);
            return;
        }

        lastUpdateTime = result.getTimestampSeconds();

        synchronized (poseEstimator) {
            robotPose = poseEstimator.getEstimatedPosition();
        }

        ATCameraResult filterResults = multiRes.estimatedPose.isPresent ?
                processMultiTarget(robotPose, multiRes) :
                processSingleTarget(robotPose, result);

        if (filterResults == null) {
            return;
        }

        int numTargets = filterResults.targetList.size();
        SafeAKitLogger.recordOutput(prefix + "/New Frame", true);
        SafeAKitLogger.recordOutput(prefix + "/Best/Pose", filterResults.best.pose);
        SafeAKitLogger.recordOutput(prefix + "/Best/Error", filterResults.best.error);
        SafeAKitLogger.recordOutput(prefix + "/Best/Pose", filterResults.alt.pose);
        SafeAKitLogger.recordOutput(prefix + "/Best/Error", filterResults.alt.error);
        SafeAKitLogger.recordOutput(prefix + "/Multitag", numTargets > 1);
        SafeAKitLogger.recordOutput(prefix + "/Number of Targets", numTargets);
        SafeAKitLogger.recordOutput(prefix + "/Ambiguity", filterResults.ambiguity);
        SafeAKitLogger.recordOutput(prefix + "/Apriltag IDs", filterResults.targetList.toString());

        var camPose = filterResults.best.pose;
        var camRot = camPose.getRotation();

        boolean updateNotFlat =
                Math.abs(camPose.getZ()) > VisionConstants.MAX_HEIGHT_METERS ||
                Math.abs(camRot.getX()) > VisionConstants.MAX_ROLL_PITCH ||
                Math.abs(camRot.getY()) > VisionConstants.MAX_ROLL_PITCH;

        boolean updateNotInField =
                camPose.getX() < 0 || camPose.getX() > VisionConstants.FIELD_LENGTH ||
                camPose.getY() < 0 || camPose.getY() > VisionConstants.FIELD_WIDTH;

        boolean updateTooAmbiguous =
                filterResults.ambiguity > VisionConstants.MAX_VALID_AMBIGUITY ||
                filterResults.best.error > VisionConstants.MAX_VALID_REPROJECTION_ERROR;

        SafeAKitLogger.recordOutput(prefix + "/NotFlat", updateNotFlat);
        SafeAKitLogger.recordOutput(prefix + "/NotInField", updateNotInField);
        SafeAKitLogger.recordOutput(prefix + "/TooAmbiguous", updateTooAmbiguous);

        if (updateNotFlat || updateNotInField || updateTooAmbiguous) {
            return;
        }

        // Single tag results are not very trustworthy. Do not use headings from them
        Pose2d camPose2d = camPose.toPose2d();
        if (numTargets == 1) {
            camPose2d = new Pose2d(camPose2d.getTranslation(), robotPose.getRotation());
        }

        synchronized (fieldPose) {
            fieldPose.getObject(cameraSettings.cameraName).setPose(camPose2d);
        }

        synchronized (poseEstimator) {
            poseEstimator.addVisionMeasurement(
                    camPose2d,
                    result.getTimestampSeconds(),
                    filterResults.stds
            );
            updates++;
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

    private record ATCameraResult(
            Vector<N3> stds,
            ATCameraResultPose best,
            ATCameraResultPose alt,
            double ambiguity,
            double distance,
            List<Integer> targetList
    ) {
        private record ATCameraResultPose(
                Pose3d pose,
                double error
        ) {}
    }
}