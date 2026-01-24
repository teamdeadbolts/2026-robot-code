/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/** Singleton class to hold the robot state */
public class RobotState {
    /* Tuning values */
    SavedLoggedNetworkNumber wheelTransStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/WheelTransStdDev", 0.05);
    SavedLoggedNetworkNumber wheelHeadingStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/WheelHeadingStdDev", 0.05);

    /* Vision base std dev (sacles with distance) */
    SavedLoggedNetworkNumber visionTransStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/VisionTransStdDev", 0.05);
    SavedLoggedNetworkNumber visionHeadingStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/VisionHeadingStdDev", 0.05);

    // private Pose3d robotPose = new Pose3d(); // Field pose of the robot
    private SwerveDrivePoseEstimator3d poseEstimator3d;
    private ChassisSpeeds fieldRelativeVelocities = new ChassisSpeeds(); // Speed of the robot field rel
    private ChassisSpeeds robotRelativeVelocities = new ChassisSpeeds(); // Speed of the robot robot rel
    private char activeAlliance = 0;
    private boolean isTransitionShift = true;
    private Timer activeTimer = new Timer();

    private static RobotState INSTANCE = new RobotState();

    private RobotState() {}

    public static RobotState getInstance() {
        return INSTANCE;
    }

    public void initPoseEstimator(Rotation3d initialRotation, SwerveModulePosition[] initalPositions) {
        this.poseEstimator3d = new SwerveDrivePoseEstimator3d(
                SwerveConstants.SWERVE_KINEMATICS,
                initialRotation,
                initalPositions,
                new Pose3d(),
                VecBuilder.fill(
                        wheelTransStdDev.get(),
                        wheelTransStdDev.get(),
                        wheelTransStdDev.get(),
                        wheelHeadingStdDev.get()),
                VecBuilder.fill(
                        visionTransStdDev.get(),
                        visionTransStdDev.get(),
                        visionTransStdDev.get(),
                        visionHeadingStdDev.get()));
    }

    /*
     =========================== GETTERS =========================
    */

    /**
     * Get the robot pose
     *
     * @return The robot pose
     */
    public Pose3d getRobotPose() {
        return poseEstimator3d.getEstimatedPosition();
    }

    /**
     * Get the robot velocities
     *
     * @return The robot velocities
     */
    public ChassisSpeeds getFieldRelativeRobotVelocities() {
        return this.fieldRelativeVelocities;
    }

    public ChassisSpeeds getRobotRelativeRobotVelocities() {
        return this.robotRelativeVelocities;
    }

    public Optional<Alliance> getActiveAlliance() {
        if (activeAlliance == 'R') return Optional.of(Alliance.Red);
        else if (activeAlliance == 'B') return Optional.of(Alliance.Blue);
        else return Optional.empty();
    }

    public double getTimeUntilActiveSwitch() {
        if (activeAlliance == 0 || isTransitionShift) return -1;

        return 25 - activeTimer.get();
    }

    public Timer getActiveTimer() {
        return this.activeTimer;
    }

    public void resetTimer() {
        this.activeTimer.reset();
        this.activeTimer.start();
    }

    /**
     * Set the robot pose
     *
     * @param newPose The new robot pose
     */
    public void setEstimatedPose(Pose3d newPose) {
        this.poseEstimator3d.resetPose(newPose);
    }

    public void setRobotRelativeVelocities(ChassisSpeeds newVelocities) {
        this.robotRelativeVelocities = newVelocities;
    }

    public void setFieldRelativeVelocities(ChassisSpeeds newVelocities) {
        this.fieldRelativeVelocities = newVelocities;
    }

    public void updateFromSwerve(SwerveModulePosition[] positions, Rotation3d gyroRotation) {
        poseEstimator3d.update(gyroRotation, positions);
    }

    public void updateActiveAlliance() {
        if (activeAlliance == 0 && !DriverStation.getGameSpecificMessage().isBlank()) {
            activeAlliance = DriverStation.getGameSpecificMessage().charAt(0);
        }

        if (isTransitionShift && activeTimer.hasElapsed(10)) {
            isTransitionShift = false;
            activeTimer.reset();
        }

        if (activeTimer.hasElapsed(25)) {
            activeTimer.reset();
            if (activeAlliance == 'R') activeAlliance = 'B';
            else activeAlliance = 'R';
        }
    }

    public void addVisionMeasurement(Pose3d visionPose, double timestamp, double distance) {
        double transStdDev = visionTransStdDev.get() * distance;
        double headingStdDev = visionHeadingStdDev.get() * distance;

        poseEstimator3d.addVisionMeasurement(
                visionPose, timestamp, VecBuilder.fill(transStdDev, transStdDev, transStdDev, headingStdDev));
    }
}
