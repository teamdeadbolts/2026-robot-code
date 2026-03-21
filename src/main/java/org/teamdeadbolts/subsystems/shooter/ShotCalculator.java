/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.TimedExtrapolatingDoubleMap;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

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

        public Pose3d virtTarget;

        @Override
        public String toString() {
            return String.format(
                    "ShotParameters{hoodAngle=%.2f, turretAngle=%.2f, wheelSpeed=%.2f, ballVelocity=%.2f}",
                    hoodAngle, turretAngle, wheelSpeed, ballVelocity);
        }
    }

    private static final double G = 9.81;

    public static double shooterMPSToRPM(final double mps) {
        return ShooterConstants.SHOOTER_MPS_TO_RPM_MAP.get(mps);
        // double smallWheelCircumference = Units.inchesToMeters(1) * 2 * Math.PI;
        // double largeWheelCircumference = Units.inchesToMeters(1.5) * 2 * Math.PI;
        // double wheelSpeed = mps / (smallWheelCircumference + largeWheelCircumference);
        // return wheelSpeed * 60.0;
    }

    private static double calculateFieldRelativeTurret(final Pose2d robotPose, final Translation2d targetLocation) {
        final Transform2d turretOffset =
                new Transform2d(ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(), new Rotation2d());
        final Pose2d turretFieldPose = robotPose.transformBy(turretOffset);

        final Translation2d relativeTrans = targetLocation.minus(turretFieldPose.getTranslation());
        final Rotation2d fieldRelativeAngle = new Rotation2d(relativeTrans.getX(), relativeTrans.getY());

        final Rotation2d robotRelAngle = fieldRelativeAngle.minus(robotPose.getRotation());
        return MathUtil.inputModulus(robotRelAngle.getRadians(), -Math.PI, Math.PI);
    }

    private static double evaluateError(final double theta, final Translation2d trans, final double alpha) {
        final double dx = trans.getX() - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(theta));
        final double dy = trans.getY() - (ShooterConstants.EXIT_RADIUS_METERS * Math.sin(theta));
        return Math.tan(theta) - ((2 * dy / dx) + Math.tan(alpha));
    }

    private static double calculateVel(final double theta, final Translation2d trans) {
        final double dx = trans.getX() - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(theta));
        final double dy = trans.getY() - (ShooterConstants.EXIT_RADIUS_METERS * Math.sin(theta));
        final double denom = 2 * Math.pow(Math.cos(theta), 2) * (dx * Math.tan(theta) - dy);
        return (denom <= 0) ? 0 : Math.sqrt((G * Math.pow(dx, 2)) / denom);
    }

    private Translation2d lastAcceptedVirtTarget2d = null;
    private ShotParametersAutoLogged lastAcceptedShot = null;
    /* --- Tuning Parameters --- */
    private final SavedTunableNumber calcIterations = SavedTunableNumber.get("Tuning/Shooter/CalcIterations", 30);
    private final SavedTunableNumber minImpactAngle =
            SavedTunableNumber.get("Tuning/Shooter/MinImpactAngleDegrees", 10);

    private final SavedTunableNumber maxImpactAngle =
            SavedTunableNumber.get("Tuning/Shooter/MaxImpactAngleDegrees", 45);
    private final SavedTunableNumber shootLatancyMs = SavedTunableNumber.get("Tuning/Shooter/ShootLatancyMs", 0);
    private final SavedTunableNumber timeToKeepVel = SavedTunableNumber.get("Tuning/Shooter/TimeToKeepVelMs", 1000);

    private final SavedTunableNumber linerFilter = SavedTunableNumber.get("Tuning/Shooter/LinearFilter", 5);
    private final TimedExtrapolatingDoubleMap vxMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vyMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());

    private final TimedExtrapolatingDoubleMap vthetaMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());

    private LinearFilter vxFilter = LinearFilter.movingAverage(linerFilter.get().intValue());

    private LinearFilter vyFilter = LinearFilter.movingAverage(linerFilter.get().intValue());

    private LinearFilter vthetaFilter =
            LinearFilter.movingAverage(linerFilter.get().intValue());

    public ShotCalculator() {
        linerFilter.addRefreshable(this);
    }

    @Override
    public void refresh() {
        vxFilter = LinearFilter.movingAverage(linerFilter.get().intValue());
        vyFilter = LinearFilter.movingAverage(linerFilter.get().intValue());
        vthetaFilter = LinearFilter.movingAverage(linerFilter.get().intValue());
    }

    /**
     * Records robot velocity state for motion compensation.
     * @param timestamp Current FPGA timestamp.
     * @param speeds Current field-relative chassis speeds.
     */
    public void updateVelocityState(final double timestamp, final ChassisSpeeds speeds) {
        final double smoothedVx = vxFilter.calculate(speeds.vxMetersPerSecond);
        final double smoothedVy = vyFilter.calculate(speeds.vyMetersPerSecond);
        final double smoothedVtheta = vthetaFilter.calculate(speeds.omegaRadiansPerSecond);

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
            final Pose3d robotPose,
            final Translation3d target,
            final double currentTime,
            final double tolerance,
            final double maxAngle) {

        // 1. Predict robot velocities at shot release
        final double latencySec = shootLatancyMs.get() / 1000.0;
        final double pVx = vxMap.get(currentTime + latencySec);
        final double pVy = vyMap.get(currentTime + latencySec);
        final double pVtheta = vthetaMap.get(currentTime + latencySec);

        // 2. Predict the robot's pose at the exact moment the ball leaves the shooter
        final Pose2d currentPose2d = robotPose.toPose2d();
        final Pose2d predictedPose2d = new Pose2d(
                currentPose2d.getX() + (pVx * latencySec),
                currentPose2d.getY() + (pVy * latencySec),
                new Rotation2d(currentPose2d.getRotation().getRadians() + (pVtheta * latencySec)));

        // Elevate back to 3D for turret math
        final Pose3d predictedRobotPose = new Pose3d(
                predictedPose2d.getX(),
                predictedPose2d.getY(),
                robotPose.getZ(),
                new edu.wpi.first.math.geometry.Rotation3d(
                        0, 0, predictedPose2d.getRotation().getRadians()));

        // 3. Calculate turret geometry based on the PREDICTED pose
        final Pose3d fieldRelTurret = predictedRobotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);
        final Translation2d turretPos2d = fieldRelTurret.getTranslation().toTranslation2d();

        final Rotation2d robotAngle = predictedPose2d.getRotation();
        final Translation2d robotRelOffset =
                ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
        final Translation2d fieldRelOffset = robotRelOffset.rotateBy(robotAngle);

        // Calculate turret velocity at the predicted moment
        // Note: Cross product of omega and radius for tangential velocity
        final double tVx = pVx - (pVtheta * fieldRelOffset.getY());
        final double tVy = pVy + (pVtheta * fieldRelOffset.getX());

        final Translation2d turretVel = new Translation2d(tVx, tVy);

        Translation2d virtTarget2d = target.toTranslation2d();
        final double targetZ = target.getZ();

        final double minAlphaRad = Math.toRadians(minImpactAngle.get());
        final double maxAlphaRad = Math.toRadians(maxImpactAngle.get());

        double hoodAngle = 0.0;
        double ballVelocity = 0.0;
        double impactAngle = 0.0;
        double distFromPivotToTarget = 0.0;
        final double heightFromPivotToTarget = targetZ - fieldRelTurret.getZ();

        // 4. Iteratively solve for flight time, target lead, and optimal impact angle
        for (int i = 0; i < calcIterations.get().intValue(); i++) {
            distFromPivotToTarget = turretPos2d.getDistance(virtTarget2d);
            final Translation2d relVirtTarget = new Translation2d(distFromPivotToTarget, heightFromPivotToTarget);

            // Estimate true dx/dy by accounting for exit radius using previous hood angle
            double dx = Math.max(0.01, distFromPivotToTarget); // Prevent division by zero
            double dy = heightFromPivotToTarget;
            if (i > 0) {
                dx = Math.max(0.01, dx - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(hoodAngle)));
                dy -= ShooterConstants.EXIT_RADIUS_METERS * Math.sin(hoodAngle);
            }

            // Calculate optimal impact angle to minimize velocity
            final double k = dy / dx;
            final double idealAlphaRad = Math.atan(Math.sqrt(k * k + 1) - k);

            // Clamp within our physical/strategic bounds
            impactAngle = MathUtil.clamp(idealAlphaRad, minAlphaRad, maxAlphaRad);

            hoodAngle = findLaunchAngle(relVirtTarget, impactAngle, maxAngle);
            ballVelocity = calculateVel(hoodAngle, relVirtTarget);

            final double v0x = ballVelocity * Math.cos(hoodAngle);

            // Only shift the virtual target by the FLIGHT time, because we already handled latency by moving the robot
            final double flightTimeSec = (distFromPivotToTarget / v0x);

            virtTarget2d = target.toTranslation2d().minus(turretVel.times(flightTimeSec));
        }

        // Use the predicted pose to calculate the final turret angle
        final double turretAngle = calculateFieldRelativeTurret(predictedPose2d, virtTarget2d);

        final ShotParametersAutoLogged rawShot = new ShotParametersAutoLogged();
        rawShot.hoodAngle = Math.PI / 2 - hoodAngle;
        rawShot.turretAngle = turretAngle;
        rawShot.ballVelocity = ballVelocity;
        rawShot.rawLaunchAngleRad = hoodAngle;
        rawShot.impactAngle = impactAngle;
        rawShot.virtTarget = new Pose3d(virtTarget2d.getX(), virtTarget2d.getY(), targetZ, new Rotation3d());
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

        final ShotParametersAutoLogged shotToUse = acceptedNew ? rawShot : lastAcceptedShot;
        if (acceptedNew) {
            lastAcceptedShot = rawShot;
            lastAcceptedVirtTarget2d = virtTarget2d;
        }

        shotToUse.wheelSpeed = shooterMPSToRPM(shotToUse.ballVelocity);
        Logger.processInputs("ShotCalc/RealShot", shotToUse);

        rawShot.isPossible = 5000.0 >= shotToUse.wheelSpeed;

        return rawShot;
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
            final Pose2d robotPose, final Translation2d targetLocation, final double currentTime) {

        // Get latency in ms and convert to seconds for math
        final double latencyMs = shootLatancyMs.get();
        final double latencySec = latencyMs / 1000.0;

        // Look up the predicted velocities at (currentTime + latencySec)
        final double predictedVx = vxMap.get(currentTime + latencySec);
        final double predictedVy = vyMap.get(currentTime + latencySec);
        final double predictedOmega = vthetaMap.get(currentTime + latencySec);

        // Calculate predicted pose: Current Position + (Velocity * Time)
        final Pose2d predictedRobotPose = new Pose2d(
                new Translation2d(
                        robotPose.getX() + (predictedVx * latencySec), robotPose.getY() + (predictedVy * latencySec)),
                new Rotation2d(robotPose.getRotation().getRadians() + (predictedOmega * latencySec)));

        return calculateFieldRelativeTurret(predictedRobotPose, targetLocation);
    }

    private double findLaunchAngle(final Translation2d trans, final double alpha, final double maxAngle) {
        double low = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES);
        double high = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES);

        for (int i = 0; i < calcIterations.get().intValue(); i++) {
            final double mid = (low + high) / 2;
            if (evaluateError(mid, trans, alpha) > 0) high = mid;
            else low = mid;
        }
        return (low + high) / 2;
    }
}
