/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO.PoseObservation;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkBoolean;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Manages computer vision data processing for robot localization.
 * Fuses camera observations with the field layout to provide pose estimates
 * to the {@link RobotState}.
 */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonVisionIOCtxAutoLogged[] ctxs;
    private final PhotonVisionIO[] ios;

    /* --- Tuning Parameters --- */
    private final SavedLoggedNetworkNumber maxAmbiguity =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/MaxAmbiguity", 0.5);
    private final SavedLoggedNetworkNumber maxTagDist =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/MaxTagDist", 0.5);
    private final SavedLoggedNetworkBoolean enableVision =
            SavedLoggedNetworkBoolean.get("Tuning/PoseEstimator/EnableVision", true);

    private final ArrayList<Pose3d> tagPoses = new ArrayList<>();
    private final ArrayList<Pose3d> robotPoses = new ArrayList<>();
    private int loopCount = 0;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem, PhotonVisionIO... ios) {
        this.ios = ios;
        this.ctxs = new PhotonVisionIOCtxAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            this.ctxs[i] = new PhotonVisionIOCtxAutoLogged();
        }
    }

    public PhotonVisionIO[] getCameras() {
        return this.ios;
    }

    @Override
    public void periodic() {
        long startTime = RobotController.getFPGATime();
        Tracer tracer = new Tracer();

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
            for (int tId : ctxs[index].tagIds) {
                VisionConstants.FIELD_LAYOUT.getTagPose(tId).ifPresent(tagPoses::add);
            }

            tracer.addEpoch("Camera" + index + " TagPoses");

            // Filter and process camera observations
            for (PoseObservation observation : ctxs[index].observations) {
                boolean isInsideField = observation.pose().getX() > 0.0
                        && observation.pose().getX() < VisionConstants.FIELD_LAYOUT.getFieldLength()
                        && observation.pose().getY() > 0.0
                        && observation.pose().getY() < VisionConstants.FIELD_LAYOUT.getFieldWidth();

                boolean acceptPose = observation.ambiguity() <= maxAmbiguity.get()
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

            // // Logging telemetry
            // if (loopCount++ > 4) {
            //     loopCount = 0;
            //     Logger.recordOutput(
            //             "Vision/Camera " + ios[index].getName() + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            //     Logger.recordOutput(
            //             "Vision/Camera " + ios[index].getName() + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            // }
        }

        // Performance monitoring
        long elapsed = RobotController.getFPGATime() - startTime;
        if (elapsed > 1_000_000) {
            tracer.printEpochs();
        }

        Logger.recordOutput("State/Pose", RobotState.getInstance().getRobotPose());
    }
}
