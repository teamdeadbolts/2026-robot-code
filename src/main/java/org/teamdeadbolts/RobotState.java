/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Singleton service that maintains the canonical state of the robot.
 * Manages pose estimation, field-relative velocities, alliance-specific
 * logic, and vision measurement fusion.
 */
public class RobotState {
    /* Tuning values */
    private final SavedLoggedNetworkNumber wheelTransStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/WheelTransStdDev", 0.05);
    private final SavedLoggedNetworkNumber wheelHeadingStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/WheelHeadingStdDev", 0.05);

    /* Vision base std dev (scales with distance) */
    private final SavedLoggedNetworkNumber visionTransStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/VisionTransStdDev", 0.05);
    private final SavedLoggedNetworkNumber visionHeadingStdDev =
            SavedLoggedNetworkNumber.get("Tuning/PoseEstimator/VisionHeadingStdDev", 0.05);

    private final Matrix<N4, N1> visionStdDevs = VecBuilder.fill(0, 0, 0, 0);

    private SwerveDrivePoseEstimator3d poseEstimator3d;
    private ChassisSpeeds fieldRelativeVelocities = new ChassisSpeeds();
    private ChassisSpeeds robotRelativeVelocities = new ChassisSpeeds();
    private char activeAlliance = 0;
    private boolean isTransitionShift = true;
    private final Timer activeTimer = new Timer();

    private static final RobotState INSTANCE = new RobotState();

    private RobotState() {}

    /** @return The singleton instance of the RobotState. */
    public static RobotState getInstance() {
        return INSTANCE;
    }

    /**
     * Initializes the pose estimator with starting rotation and module positions.
     *
     * @param initialRotation The gyro rotation at startup.
     * @param initalPositions The initial swerve module positions.
     */
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

    /** @return The estimated field-relative robot pose. */
    public Pose3d getRobotPose() {
        return poseEstimator3d.getEstimatedPosition();
    }

    /** @return Robot velocities in the field-relative frame. */
    public ChassisSpeeds getFieldRelativeRobotVelocities() {
        return this.fieldRelativeVelocities;
    }

    /** @return Robot velocities in the robot-relative frame. */
    public ChassisSpeeds getRobotRelativeRobotVelocities() {
        return this.robotRelativeVelocities;
    }

    /** @return The currently active alliance. */
    public Optional<Alliance> getActiveAlliance() {
        if (activeAlliance == 'R') return Optional.of(Alliance.Red);
        else if (activeAlliance == 'B') return Optional.of(Alliance.Blue);
        else return Optional.empty();
    }

    /** @return Time remaining in seconds for the current alliance switch cycle. */
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
     * Resets the pose estimator to a new pose.
     *
     * @param newPose The new pose to set.
     */
    public void setEstimatedPose(Pose3d newPose) {
        this.poseEstimator3d.resetPose(newPose);
    }

    public void setRobotRelativeVelocities(ChassisSpeeds newVelocities) {
        Logger.recordOutput("State/RobotRelativeVelocites", newVelocities);
        this.robotRelativeVelocities = newVelocities;
    }

    public void setFieldRelativeVelocities(ChassisSpeeds newVelocities) {
        Logger.recordOutput("State/FieldRelativeVelocites", newVelocities);
        this.fieldRelativeVelocities = newVelocities;
    }

    /**
     * Updates the pose estimator with drivetrain data.
     *
     * @param positions    Current swerve module positions.
     * @param gyroRotation Current gyroscope rotation.
     */
    public void updateFromSwerve(SwerveModulePosition[] positions, Rotation3d gyroRotation) {
        poseEstimator3d.update(gyroRotation, positions);
    }

    /**
     * Updates which alliance hub is active
     */
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

    /**
     * Fuses a vision measurement into the pose estimator, scaling confidence by
     * distance.
     *
     * @param visionPose The pose reported by vision.
     * @param timestamp  The timestamp of the measurement.
     * @param distance   The distance from the target to scale uncertainty.
     */
    public void addVisionMeasurement(Pose3d visionPose, double timestamp, double distance) {
        double transStdDev = visionTransStdDev.get() * distance;
        double headingStdDev = visionHeadingStdDev.get() * distance;

        visionStdDevs.set(0, 0, transStdDev);
        visionStdDevs.set(1, 0, transStdDev);
        visionStdDevs.set(2, 0, transStdDev);
        visionStdDevs.set(3, 0, headingStdDev);

        poseEstimator3d.addVisionMeasurement(visionPose, timestamp, visionStdDevs);
    }
}
