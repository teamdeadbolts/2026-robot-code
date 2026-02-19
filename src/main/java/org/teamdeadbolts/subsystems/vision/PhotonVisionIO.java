/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.teamdeadbolts.constants.VisionConstants;

public class PhotonVisionIO {
    private final PhotonCamera camera;
    private Transform3d offset = null;
    private Supplier<Transform3d> offsetSupplier = null;
    private final Map<Integer, Pose3d> tagPoseCache = new HashMap<>();

    private HashSet<Integer> tagIds = new HashSet<>(8);
    private ArrayList<PoseObservation> poseObservations = new ArrayList<>(8);

    /**
     * @param camName The name of the PhotonCamera.
     * @param offset The Transform3d from the Robot's center to the Camera's lens.
     * @param fieldLayout The loaded AprilTagFieldLayout for the current competition.
     */
    public PhotonVisionIO(String camName, Transform3d offset) {
        this.camera = new PhotonCamera(camName);
        this.offset = offset;

        for (AprilTag tag : VisionConstants.FIELD_LAYOUT.getTags()) {
            tagPoseCache.put(tag.ID, tag.pose);
        }
    }

    public PhotonVisionIO(String camName, Supplier<Transform3d> offsetSupplier) {
        this.camera = new PhotonCamera(camName);
        this.offsetSupplier = offsetSupplier;

        for (AprilTag tag : VisionConstants.FIELD_LAYOUT.getTags()) {
            tagPoseCache.put(tag.ID, tag.pose);
        }
    }

    public void update(PhotonVisionIOCtx ctx) {
        Tracer tracer = new Tracer();
        long startTime = RobotController.getFPGATime();
        Transform3d currOffset = this.offset;
        if (offsetSupplier != null) {
            currOffset = offsetSupplier.get();
        }

        tracer.addEpoch("Start");

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        tracer.addEpoch("Get Results");

        if (results.isEmpty()) {
            ctx.observations = new PoseObservation[0];
            ctx.tagIds = new int[0];
            tracer.addEpoch("No Results");
        } else {

            PhotonPipelineResult result = results.get(results.size() - 1);

            tagIds.clear();
            poseObservations.clear();
            tracer.addEpoch("Clear Lists");

            if (result.hasTargets()) {
                PhotonTrackedTarget best = result.getBestTarget();
                Pose3d tagPose = tagPoseCache.get(best.getFiducialId());

                tracer.addEpoch("Tag lookup");
                if (tagPose != null) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
                    Transform3d camToTarget = best.bestCameraToTarget;
                    Transform3d fieldToCam = fieldToTarget.plus(camToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCam.plus(currOffset.inverse());

                    tracer.addEpoch("Pose calcs");

                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    tagIds.add(best.fiducialId);
                    poseObservations.add(new PoseObservation(
                            result.getTimestampSeconds(),
                            robotPose,
                            best.poseAmbiguity,
                            camToTarget.getTranslation().getNorm()));

                    tracer.addEpoch("Add Observation");
                }
            }

            ctx.observations = poseObservations.toArray(new PoseObservation[poseObservations.size()]);

            ctx.tagIds = new int[tagIds.size()];
            int i = 0;
            for (int tagId : tagIds) {
                ctx.tagIds[i++] = tagId;
            }
            tracer.addEpoch("Outputs");
        }

        long elapsed = RobotController.getFPGATime() - startTime;
        if (elapsed > 300_000) {
            tracer.printEpochs();
        }
    }

    @AutoLog
    public static class PhotonVisionIOCtx {
        public PoseObservation[] observations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity, double tagDist) {}
}
