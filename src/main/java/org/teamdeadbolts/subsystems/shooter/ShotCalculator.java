/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.TimedExtrapolatingDoubleMap;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Calculates optimal shooter parameters (hood angle, wheel speed, turret angle)
 * to land a game piece in a target, accounting for robot motion, ball flight
 * time, air resistance, and optimal impact angle to minimize RPM.
 */
public class ShotCalculator implements Refreshable {
    @AutoLog
    public static class ShotParameters {
        public double hoodAngle;
        public double turretAngle;
        public double wheelSpeed;
        public double impactAngle;

        public double ballVelocity;
        public double rawLaunchAngleRad;

        public boolean isPossible;

        public Translation3d virtTarget;

        @Override
        public String toString() {
            return String.format(
                    "ShotParameters{hoodAngle=%.2f, turretAngle=%.2f, wheelSpeed=%.2f, ballVelocity=%.2f}",
                    hoodAngle, turretAngle, wheelSpeed, ballVelocity);
        }
    }

    private Translation2d lastAcceptedVirtTarget2d = null;
    private ShotParametersAutoLogged lastAcceptedShot = null;

    private static final double G = 9.81;

    /* --- Tuning Parameters --- */
    private final SavedLoggedNetworkNumber calcIterations =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/CalcIterations", 30);
    private final SavedLoggedNetworkNumber minImpactAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/MinImpactAngleDegrees", 10);
    private final SavedLoggedNetworkNumber maxImpactAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/MaxImpactAngleDegrees", 45);
    private final SavedLoggedNetworkNumber shootLatancyMs =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShootLatancyMs", 0);
    private final SavedLoggedNetworkNumber timeToKeepVel =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TimeToKeepVelMs", 1000);
    private final SavedLoggedNetworkNumber airResistanceMultiplier =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/airResistanceMultiplier", 0.01);

    private final SavedLoggedNetworkNumber linerFilter = SavedLoggedNetworkNumber.get("Tuning/Shooter/LinearFilter", 5);

    private final TimedExtrapolatingDoubleMap vxMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vyMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vthetaMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());

    private LinearFilter vxFilter = LinearFilter.movingAverage((int) linerFilter.get());
    private LinearFilter vyFilter = LinearFilter.movingAverage((int) linerFilter.get());
    private LinearFilter vthetaFilter = LinearFilter.movingAverage((int) linerFilter.get());

    public ShotCalculator() {
        linerFilter.addRefreshable(this);
    }

    @Override
    public void refresh() {
        vxFilter = LinearFilter.movingAverage((int) linerFilter.get());
        vyFilter = LinearFilter.movingAverage((int) linerFilter.get());
        vthetaFilter = LinearFilter.movingAverage((int) linerFilter.get());
    }

    /**
     * Records robot velocity state for motion compensation.
     * @param timestamp Current FPGA timestamp.
     * @param speeds Current field-relative chassis speeds.
     */
    public void updateVelocityState(double timestamp, ChassisSpeeds speeds) {
        double smoothedVx = vxFilter.calculate(speeds.vxMetersPerSecond);
        double smoothedVy = vyFilter.calculate(speeds.vyMetersPerSecond);
        double smoothedVtheta = vthetaFilter.calculate(speeds.omegaRadiansPerSecond);

        vxMap.put(timestamp, smoothedVx);
        vyMap.put(timestamp, smoothedVy);
        vthetaMap.put(timestamp, smoothedVtheta);
    }

    /**
     * Calculates shot parameters based on target position and robot motion.
     * Uses iterative solving to compensate for target movement during flight time.
     *
     * @param robotPose Current field-relative robot pose.
     * @param target Field-relative 3D target coordinates.
     * @param currentTime Current timestamp.
     * @param tolerance Distance threshold for updating target aim.
     * @return Calculated shot parameters.
     */
    /**
     * Calculates shot parameters based on target position and robot motion.
     * Uses iterative solving to compensate for target movement during flight time.
     *
     * @param robotPose Current field-relative robot pose.
     * @param target Field-relative 3D target coordinates.
     * @param currentTime Current timestamp.
     * @param tolerance Distance threshold for updating target aim.
     * @return Calculated shot parameters.
     */
    public ShotParametersAutoLogged calculateShot(
            Pose3d robotPose, Translation3d target, double currentTime, double tolerance, double maxAngle) {

        // 1. Predict robot velocities at shot release
        double latencySec = shootLatancyMs.get() / 1000.0;
        double pVx = vxMap.get(currentTime + latencySec);
        double pVy = vyMap.get(currentTime + latencySec);
        double pVtheta = vthetaMap.get(currentTime + latencySec);

        // 2. Predict the robot's pose at the exact moment the ball leaves the shooter
        Pose2d currentPose2d = robotPose.toPose2d();
        Pose2d predictedPose2d = new Pose2d(
                currentPose2d.getX() + (pVx * latencySec),
                currentPose2d.getY() + (pVy * latencySec),
                new Rotation2d(currentPose2d.getRotation().getRadians() + (pVtheta * latencySec)));

        // Elevate back to 3D for turret math
        Pose3d predictedRobotPose = new Pose3d(
                predictedPose2d.getX(),
                predictedPose2d.getY(),
                robotPose.getZ(),
                new edu.wpi.first.math.geometry.Rotation3d(
                        0, 0, predictedPose2d.getRotation().getRadians()));

        // 3. Calculate turret geometry based on the PREDICTED pose
        Pose3d fieldRelTurret = predictedRobotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);
        Translation2d turretPos2d = fieldRelTurret.getTranslation().toTranslation2d();

        Rotation2d robotAngle = predictedPose2d.getRotation();
        Translation2d robotRelOffset =
                ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
        Translation2d fieldRelOffset = robotRelOffset.rotateBy(robotAngle);

        // Calculate turret velocity at the predicted moment
        // Note: Cross product of omega and radius for tangential velocity
        double tVx = pVx - (pVtheta * fieldRelOffset.getY());
        double tVy = pVy + (pVtheta * fieldRelOffset.getX());

        Translation2d turretVel = new Translation2d(tVx, tVy);

        Translation2d virtTarget2d = target.toTranslation2d();
        double targetZ = target.getZ();

        double minAlphaRad = Math.toRadians(minImpactAngle.get());
        double maxAlphaRad = Math.toRadians(maxImpactAngle.get());

        double hoodAngle = 0.0;
        double ballVelocity = 0.0;
        double impactAngle = 0.0;
        double distFromPivotToTarget = 0.0;
        double heightFromPivotToTarget = targetZ - fieldRelTurret.getZ();

        // 4. Iteratively solve for flight time, target lead, and optimal impact angle
        for (int i = 0; i < (int) calcIterations.get(); i++) {
            distFromPivotToTarget = turretPos2d.getDistance(virtTarget2d);
            Translation2d relVirtTarget = new Translation2d(distFromPivotToTarget, heightFromPivotToTarget);

            // Estimate true dx/dy by accounting for exit radius using previous hood angle
            double dx = Math.max(0.01, distFromPivotToTarget); // Prevent division by zero
            double dy = heightFromPivotToTarget;
            if (i > 0) {
                dx = Math.max(0.01, dx - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(hoodAngle)));
                dy -= ShooterConstants.EXIT_RADIUS_METERS * Math.sin(hoodAngle);
            }

            // Calculate optimal impact angle to minimize velocity
            double k = dy / dx;
            double idealAlphaRad = Math.atan(Math.sqrt(k * k + 1) - k);

            // Clamp within our physical/strategic bounds
            impactAngle = MathUtil.clamp(idealAlphaRad, minAlphaRad, maxAlphaRad);

            hoodAngle = findLaunchAngle(relVirtTarget, impactAngle, maxAngle);
            ballVelocity = calculateVel(hoodAngle, relVirtTarget);

            double v0x = ballVelocity * Math.cos(hoodAngle);

            // Only shift the virtual target by the FLIGHT time, because we already handled latency by moving the robot
            double flightTimeSec = (distFromPivotToTarget / v0x);

            virtTarget2d = target.toTranslation2d().minus(turretVel.times(flightTimeSec));
        }

        // Use the predicted pose to calculate the final turret angle
        double turretAngle = calculateFieldRelativeTurret(predictedPose2d, virtTarget2d);

        ShotParametersAutoLogged rawShot = new ShotParametersAutoLogged();
        rawShot.hoodAngle = Math.PI / 2 - hoodAngle;
        rawShot.turretAngle = turretAngle;
        rawShot.ballVelocity = ballVelocity;
        rawShot.rawLaunchAngleRad = hoodAngle;
        rawShot.impactAngle = impactAngle;
        rawShot.virtTarget = new Translation3d(virtTarget2d.getX(), virtTarget2d.getY(), targetZ);
        rawShot.wheelSpeed = shooterMPSToRPM(ballVelocity);

        Logger.processInputs("ShotCalc/RawShot", rawShot);

        // Apply hysteresis/tolerance check
        boolean acceptedNew = true;
        if (tolerance > 0.0 && lastAcceptedVirtTarget2d != null && lastAcceptedShot != null) {
            if (virtTarget2d.minus(lastAcceptedVirtTarget2d).getNorm() <= tolerance) {
                acceptedNew = false;
                lastAcceptedShot.turretAngle = rawShot.turretAngle;
            }
        }

        ShotParametersAutoLogged shotToUse = acceptedNew ? rawShot : lastAcceptedShot;
        if (acceptedNew) {
            lastAcceptedShot = rawShot;
            lastAcceptedVirtTarget2d = virtTarget2d;
        }

        shotToUse.wheelSpeed = shooterMPSToRPM(shotToUse.ballVelocity);
        Logger.processInputs("ShotCalc/RealShot", shotToUse);

        shotToUse.isPossible = ShooterConstants.SHOOTER_MPS_TO_RPM_MAP.get(9999.0) > shotToUse.wheelSpeed;

        return shotToUse;
    }

    /**
     * Calculates the turret angle required to aim at a target, accounting for the latency
     * between the current time and when the shot will be fired.
     *
     * @param robotPose      Current field-relative robot pose.
     * @param targetLocation Field-relative coordinates of the target.
     * @return The turret angle offset required for latency compensation in radians.
     */
    public double calculateLatancyOffsetTurretAngle(
            Pose2d robotPose, Translation2d targetLocation, double currentTime) {

        // Get latency in ms and convert to seconds for math
        double latencyMs = shootLatancyMs.get();
        double latencySec = latencyMs / 1000.0;

        // Look up the predicted velocities at (currentTime + latencySec)
        double predictedVx = vxMap.get(currentTime + latencySec);
        double predictedVy = vyMap.get(currentTime + latencySec);
        double predictedOmega = vthetaMap.get(currentTime + latencySec);

        // Calculate predicted pose: Current Position + (Velocity * Time)
        Pose2d predictedRobotPose = new Pose2d(
                new Translation2d(
                        robotPose.getX() + (predictedVx * latencySec), robotPose.getY() + (predictedVy * latencySec)),
                new Rotation2d(robotPose.getRotation().getRadians() + (predictedOmega * latencySec)));

        return calculateFieldRelativeTurret(predictedRobotPose, targetLocation);
    }

    private static double calculateFieldRelativeTurret(Pose2d robotPose, Translation2d targetLocation) {
        Transform2d turretOffset =
                new Transform2d(ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(), new Rotation2d());
        Pose2d turretFieldPose = robotPose.transformBy(turretOffset);

        Translation2d relativeTrans = targetLocation.minus(turretFieldPose.getTranslation());
        Rotation2d fieldRelativeAngle = new Rotation2d(relativeTrans.getX(), relativeTrans.getY());

        Rotation2d robotRelAngle = fieldRelativeAngle.minus(robotPose.getRotation());
        return MathUtil.inputModulus(robotRelAngle.getRadians(), -Math.PI, Math.PI);
    }

    public static double shooterMPSToRPM(double mps) {
        return ShooterConstants.SHOOTER_MPS_TO_RPM_MAP.get(mps);
    }

    private double findLaunchAngle(Translation2d trans, double alpha, double maxAngle) {
        double low = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES);
        double high = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES);

        for (int i = 0; i < (int) calcIterations.get(); i++) {
            double mid = (low + high) / 2;
            if (evaluateError(mid, trans, alpha) > 0) high = mid;
            else low = mid;
        }
        return (low + high) / 2;
    }

    private static double evaluateError(double theta, Translation2d trans, double alpha) {
        double dx = trans.getX() - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(theta));
        double dy = trans.getY() - (ShooterConstants.EXIT_RADIUS_METERS * Math.sin(theta));
        return Math.tan(theta) - ((2 * dy / dx) + Math.tan(alpha));
    }

    private static double calculateVel(double theta, Translation2d trans) {
        double dx = trans.getX() - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(theta));
        double dy = trans.getY() - (ShooterConstants.EXIT_RADIUS_METERS * Math.sin(theta));
        double denom = 2 * Math.pow(Math.cos(theta), 2) * (dx * Math.tan(theta) - dy);
        return (denom <= 0) ? 0 : Math.sqrt((G * Math.pow(dx, 2)) / denom);
    }
}
