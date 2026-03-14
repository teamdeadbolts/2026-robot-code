/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * IO implementation for PhotonVision cameras. Handles hardware communication,
 * AprilTag pose estimation, and camera-to-robot coordinate transformations.
 */
public class PhotonVisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private Transform3d offset = null;
    private Supplier<Transform3d> offsetSupplier = null;
    private final Map<Integer, Pose3d> tagPoseCache = new HashMap<>();

    private final HashSet<Integer> tagIds = new HashSet<>(8);
    private final ArrayList<PoseObservation> poseObservations = new ArrayList<>(8);

    private final SavedLoggedNetworkNumber testRobotPoseX =
            SavedLoggedNetworkNumber.get("Tuning/Vision/TestRobotPoseX", 0);
    private final SavedLoggedNetworkNumber testRobotPoseY =
            SavedLoggedNetworkNumber.get("Tuning/Vision/TestRobotPoseY", 0);
    private final SavedLoggedNetworkNumber testRobotRotationDeg =
            SavedLoggedNetworkNumber.get("Tuning/Vision/TestRobotRotationDeg", 0);

    // private final SavedLoggedNetworkBoolean enableCam;
    private boolean hardDisabled = false;
    private boolean enabled = true;

    /**
     * @param camName The name of the PhotonCamera.
     * @param offset The static Transform3d from the robot's center to the camera.
     */
    public PhotonVisionIO(String camName, Transform3d offset) {
        this.camera = new PhotonCamera(camName);
        this.offset = offset;
        this.poseEstimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.offset);

        cacheTagPoses();
    }

    /**
     * @param camName The name of the PhotonCamera.
     * @param offsetSupplier Supplier for dynamic camera-to-robot transformation.
     */
    public PhotonVisionIO(String camName, Supplier<Transform3d> offsetSupplier) {
        this.camera = new PhotonCamera(camName);
        this.offsetSupplier = offsetSupplier;
        this.poseEstimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());

        cacheTagPoses();
    }

    private void cacheTagPoses() {
        for (AprilTag tag : VisionConstants.FIELD_LAYOUT.getTags()) {
            tagPoseCache.put(tag.ID, tag.pose);
        }
    }

    // Hard for network tables/tuning
    public void setHardDisabled(boolean disabled) {
        this.hardDisabled = disabled;
    }

    // For code stuff
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void findTransform() {
        Pose3d robotPose = new Pose3d(
                new Translation3d(testRobotPoseX.get(), testRobotPoseY.get(), 0),
                new Rotation3d(0, 0, Units.degreesToRadians(testRobotRotationDeg.get())));

        if (this.poseObservations.size() > 0) {
            Pose3d cameraProvidedPose = this.poseObservations.get(0).pose();
            // Calculate a Transform3d to transform the camera provided pose to the robot pose
            Transform3d currOffset = (offsetSupplier != null) ? offsetSupplier.get() : this.offset;
            Pose3d cameraFieldPose = cameraProvidedPose.transformBy(currOffset);
            Transform3d requiredOffset = cameraFieldPose.minus(robotPose);

            Logger.recordOutput(
                    "Vision/Camera " + camera.getName() + "/x",
                    requiredOffset.getTranslation().getX());
            Logger.recordOutput(
                    "Vision/Camera " + camera.getName() + "/y",
                    requiredOffset.getTranslation().getY());
            Logger.recordOutput("Vision/Camera " + camera.getName() + "/z", requiredOffset.getZ());
            Logger.recordOutput(
                    "Vision/Camera " + camera.getName() + "/yaw",
                    requiredOffset.getRotation().getZ());
            Logger.recordOutput(
                    "Vision/Camera " + camera.getName() + "/pitch",
                    requiredOffset.getRotation().getY());
        }
    }

    /**
     * Updates the provided context with the latest camera results and pose observations.
     * @param ctx The IO context to populate.
     */
    public void update(PhotonVisionIOCtx ctx) {
        if (!hardDisabled && enabled) {
            Tracer tracer = new Tracer();
            long startTime = RobotController.getFPGATime();

            if (offsetSupplier != null) poseEstimator.setRobotToCameraTransform(offsetSupplier.get());

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            tagIds.clear();
            poseObservations.clear();

            if (results.isEmpty()) {
                ctx.observations = new PoseObservation[0];
                ctx.tagIds = new int[0];
            } else {
                PhotonPipelineResult result = results.get(results.size() - 1);

                Optional<EstimatedRobotPose> estPose = poseEstimator.update(result);

                if (estPose.isPresent()) {
                    EstimatedRobotPose pose = estPose.get();
                    double totalDist = 0.0;
                    double maxAmbiguity = 0.0;

                    for (PhotonTrackedTarget target : pose.targetsUsed) {
                        tagIds.add(target.fiducialId);
                        double distanceToTag =
                                target.getBestCameraToTarget().getTranslation().getNorm();
                        totalDist += distanceToTag;

                        maxAmbiguity = Math.max(maxAmbiguity, target.poseAmbiguity);
                    }

                    double avgDist = totalDist / pose.targetsUsed.size();

                    double finalAmbiguity =
                            (pose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) ? 0 : maxAmbiguity;

                    poseObservations.add(new PoseObservation(
                            pose.timestampSeconds,
                            pose.estimatedPose,
                            finalAmbiguity,
                            avgDist,
                            enabled && !hardDisabled));
                }

                ctx.observations = poseObservations.toArray(new PoseObservation[0]);
                ctx.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();

                if (ctx.observations.length > 0) {
                    Logger.recordOutput("Vision/Camera " + getName() + "/Observations", ctx.observations);
                }
            }

            // Performance monitoring
            if (RobotController.getFPGATime() - startTime > 300_000) {
                tracer.printEpochs();
            }
        }
    }

    public String getName() {
        return camera.getName();
    }

    @AutoLog
    public static class PhotonVisionIOCtx {
        public PoseObservation[] observations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    public record PoseObservation(double timestamp, Pose3d pose, double ambiguity, double tagDist, boolean good) {}
}
