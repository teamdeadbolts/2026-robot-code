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
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

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

    private final SavedTunableNumber testRobotPoseX = SavedTunableNumber.get("Tuning/Vision/TestRobotPoseX", 0);
    private final SavedTunableNumber testRobotPoseY = SavedTunableNumber.get("Tuning/Vision/TestRobotPoseY", 0);
    private final SavedTunableNumber testRobotRotationDeg =
            SavedTunableNumber.get("Tuning/Vision/TestRobotRotationDeg", 0);

    private final SavedTunableNumber maxAmbiguity;
    private final SavedTunableNumber maxTagDist;

    // private final SavedLoggedNetworkBoolean enableCam;
    private boolean hardDisabled = false;
    private boolean enabled = true;

    /**
     * @param camName The name of the PhotonCamera.
     * @param offset The static Transform3d from the robot's center to the camera.
     */
    public PhotonVisionIO(final String camName, final Transform3d offset) {
        this.camera = new PhotonCamera(camName);
        this.offset = offset;
        this.poseEstimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.offset);

        this.maxAmbiguity = SavedTunableNumber.get("Tuning/Vision/Camera " + camName + "/MaxAmbiguity", 0.5);
        this.maxTagDist = SavedTunableNumber.get("Tuning/Vision/Camera " + camName + "/MaxTagDist", 0.5);

        cacheTagPoses();
    }

    /**
     * @param camName The name of the PhotonCamera.
     * @param offsetSupplier Supplier for dynamic camera-to-robot transformation.
     */
    public PhotonVisionIO(final String camName, final Supplier<Transform3d> offsetSupplier) {
        this.camera = new PhotonCamera(camName);
        this.offsetSupplier = offsetSupplier;
        this.poseEstimator = new PhotonPoseEstimator(
                VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());

        this.maxAmbiguity = SavedTunableNumber.get("Tuning/Vision/Camera " + camName + "/MaxAmbiguity", 0.5);
        this.maxTagDist = SavedTunableNumber.get("Tuning/Vision/Camera " + camName + "/MaxTagDist", 0.5);

        cacheTagPoses();
    }

    private void cacheTagPoses() {
        for (final AprilTag tag : VisionConstants.FIELD_LAYOUT.getTags()) {
            tagPoseCache.put(tag.ID, tag.pose);
        }
    }

    // Hard for network tables/tuning
    public void setHardDisabled(final boolean disabled) {
        this.hardDisabled = disabled;
    }

    // For code stuff
    public void setEnabled(final boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public void findTransform() {
        final Pose3d robotPose = new Pose3d(
                new Translation3d(testRobotPoseX.get(), testRobotPoseY.get(), 0),
                new Rotation3d(0, 0, Units.degreesToRadians(testRobotRotationDeg.get())));

        if (this.poseObservations.size() > 0) {
            final Pose3d cameraProvidedPose = this.poseObservations.get(0).pose();
            // Calculate a Transform3d to transform the camera provided pose to the robot pose
            final Transform3d currOffset = (offsetSupplier != null) ? offsetSupplier.get() : this.offset;
            final Pose3d cameraFieldPose = cameraProvidedPose.transformBy(currOffset);
            final Transform3d requiredOffset = cameraFieldPose.minus(robotPose);

            Logger.recordOutput(
                    "VisionSubsystem/Camera " + camera.getName() + "/x",
                    requiredOffset.getTranslation().getX());
            Logger.recordOutput(
                    "VisionSubsystem/Camera " + camera.getName() + "/y",
                    requiredOffset.getTranslation().getY());
            Logger.recordOutput("VisionSubsystem/Camera " + camera.getName() + "/z", requiredOffset.getZ());
            Logger.recordOutput(
                    "VisionSubsystem/Camera " + camera.getName() + "/yaw",
                    requiredOffset.getRotation().getZ());
            Logger.recordOutput(
                    "VisionSubsystem/Camera " + camera.getName() + "/pitch",
                    requiredOffset.getRotation().getY());
        }
    }

    /**
     * Updates the provided context with the latest camera results and pose observations.
     * @param ctx The IO context to populate.
     */
    public void update(final PhotonVisionIOCtx ctx) {
        if (!hardDisabled && enabled) {
            final Tracer tracer = new Tracer();
            final long startTime = RobotController.getFPGATime();

            if (offsetSupplier != null) poseEstimator.setRobotToCameraTransform(offsetSupplier.get());

            final List<PhotonPipelineResult> results = camera.getAllUnreadResults();

            tagIds.clear();
            poseObservations.clear();

            if (results.isEmpty()) {
                ctx.observations = new PoseObservation[0];
                ctx.tagIds = new int[0];
            } else {
                final PhotonPipelineResult result = results.get(results.size() - 1);

                final Optional<EstimatedRobotPose> estPose = poseEstimator.update(result);

                if (estPose.isPresent()) {
                    final EstimatedRobotPose pose = estPose.get();
                    double totalDist = 0.0;
                    double highestAmbiguity = 0.0;

                    for (final PhotonTrackedTarget target : pose.targetsUsed) {
                        tagIds.add(target.fiducialId);
                        final double distanceToTag =
                                target.getBestCameraToTarget().getTranslation().getNorm();
                        totalDist += distanceToTag;

                        highestAmbiguity = Math.max(highestAmbiguity, target.poseAmbiguity);
                    }

                    final double avgDist = totalDist / pose.targetsUsed.size();

                    final double finalAmbiguity =
                            (pose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) ? 0 : highestAmbiguity;

                    boolean isGood = finalAmbiguity < maxAmbiguity.get()
                            && avgDist < maxTagDist.get()
                            && enabled
                            && !hardDisabled;

                    poseObservations.add(new PoseObservation(
                            pose.timestampSeconds, pose.estimatedPose, finalAmbiguity, avgDist, isGood));
                }

                ctx.observations = poseObservations.toArray(new PoseObservation[0]);
                ctx.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();

                if (ctx.observations.length > 0) {
                    Logger.recordOutput("VisionSubsystem/Camera " + getName() + "/Observations", ctx.observations);
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
