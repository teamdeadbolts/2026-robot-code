/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO.PoseObservation;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableBoolean;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

/**
 * Manages computer vision data processing for robot localization.
 * Fuses camera observations with the field layout to provide pose estimates
 * to the {@link RobotState}.
 */
public class VisionSubsystem extends SubsystemBase implements Refreshable {

    private final PhotonVisionIOCtxAutoLogged[] ctxs;
    private final PhotonVisionIO[] ios;

    /* --- Tuning Parameters --- */
    private final SavedTunableNumber maxAmbiguity = SavedTunableNumber.get("Tuning/PoseEstimator/MaxAmbiguity", 0.5);
    private final SavedTunableNumber maxTagDist = SavedTunableNumber.get("Tuning/PoseEstimator/MaxTagDist", 0.5);
    private final SavedTunableBoolean enableVision = SavedTunableBoolean.get("Tuning/PoseEstimator/EnableVision", true);

    private final ArrayList<Pose3d> tagPoses = new ArrayList<>();
    private final ArrayList<Pose3d> robotPoses = new ArrayList<>();
    private final HashMap<PhotonVisionIO, SavedTunableBoolean> cameraToggles = new HashMap<>();
    private final Tracer tracer = new Tracer();

    public VisionSubsystem(final SwerveSubsystem swerveSubsystem, final PhotonVisionIO... ios) {
        this.ios = ios;
        this.ctxs = new PhotonVisionIOCtxAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            this.ctxs[i] = new PhotonVisionIOCtxAutoLogged();
        }

        for (final PhotonVisionIO io : this.ios) {
            final SavedTunableBoolean bool =
                    SavedTunableBoolean.get("Tuning/Vision/Camera " + io.getName() + "/Enable", true);
            this.cameraToggles.put(io, bool);
            bool.addRefreshable(this);
        }
    }

    @Override
    public void refresh() {
        this.cameraToggles.forEach((io, bool) -> {
            io.setHardDisabled(!bool.get());
        });
    }

    public PhotonVisionIO[] getCameras() {
        return this.ios;
    }

    @Override
    public void periodic() {
        final long startTime = RobotController.getFPGATime();
        tracer.resetTimer();

        tagPoses.clear();
        robotPoses.clear();

        // Update IO interfaces
        for (int i = 0; i < ios.length; i++) {
            ios[i].update(ctxs[i]);
            Logger.processInputs("Vision/Camera " + ios[i].getName() + "/Ctx", ctxs[i]);
        }

        tracer.addEpoch("Process Inputs");

        for (int index = 0; index < ctxs.length; index++) {
            // Retrieve tag poses from layout for debugging
            for (final int tId : ctxs[index].tagIds) {
                VisionConstants.FIELD_LAYOUT.getTagPose(tId).ifPresent(tagPoses::add);
            }

            tracer.addEpoch("Camera" + index + " TagPoses");

            // Filter and process camera observations
            for (final PoseObservation observation : ctxs[index].observations) {
                final boolean isInsideField = observation.pose().getX() > 0.0
                        && observation.pose().getX() < VisionConstants.FIELD_LAYOUT.getFieldLength()
                        && observation.pose().getY() > 0.0
                        && observation.pose().getY() < VisionConstants.FIELD_LAYOUT.getFieldWidth();

                final boolean acceptPose = observation.ambiguity() <= maxAmbiguity.get()
                        && observation.tagDist() <= maxTagDist.get()
                        && isInsideField;

                if (acceptPose) {
                    robotPoses.add(observation.pose());
                    if (enableVision.get() && observation.good()) {
                        RobotState.getInstance()
                                .addVisionMeasurement(
                                        observation.pose(), observation.timestamp(), observation.tagDist());
                    }
                }
            }

            tracer.addEpoch("Camera" + index + " Add Observations");
        }

        // Performance monitoring
        final long elapsed = RobotController.getFPGATime() - startTime;
        if (elapsed > 1_000_000) {
            tracer.printEpochs();
        }

        Logger.recordOutput("State/Pose", RobotState.getInstance().getRobotPose());
    }
}
