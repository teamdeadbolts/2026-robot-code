/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.TimedExtrapolatingDoubleMap;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Calculates optimal shooter parameters (hood angle, wheel speed, turret angle)
 * to land a game piece in a target, accounting for robot motion, ball flight
 * time, and air resistance.
 */
public class ShotCalculator {
    @AutoLog
    public static class ShotParameters {
        public double hoodAngle;
        public double turretAngle;
        public double wheelSpeed;
        public double ballVelocity;

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
    private static final SavedLoggedNetworkNumber calcIterations =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/CalcIterations", 3);
    private static final SavedLoggedNetworkNumber impactAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ImpactAngleDegrees", 20);
    private static final SavedLoggedNetworkNumber shootLatancyMs =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShootLatancyMs", 100);
    private static final SavedLoggedNetworkNumber timeToKeepVel =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TimeToKeepVelMs", 1000);
    private static final SavedLoggedNetworkNumber airResistanceMultiplier =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/airResistanceMultiplier", 0.01);

    private final TimedExtrapolatingDoubleMap vxMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vyMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vthetaMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());

    public ShotCalculator() {}

    /**
     * Records robot velocity state for motion compensation.
     * @param timestamp Current FPGA timestamp.
     * @param speeds Current field-relative chassis speeds.
     */
    public void updateVelocityState(double timestamp, ChassisSpeeds speeds) {
        vxMap.put(timestamp, speeds.vxMetersPerSecond);
        vyMap.put(timestamp, speeds.vyMetersPerSecond);
        vthetaMap.put(timestamp, speeds.omegaRadiansPerSecond);
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
    public ShotParametersAutoLogged calculateShot(
            Pose3d robotPose, Translation3d target, double currentTime, double tolerance, double maxAngle) {

        Pose3d fieldRelTurret = robotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);
        Translation2d turretPos2d = fieldRelTurret.getTranslation().toTranslation2d();

        // Predict robot velocity at shot release
        double pVx = vxMap.get(currentTime + shootLatancyMs.get());
        double pVy = vyMap.get(currentTime + shootLatancyMs.get());
        double pVtheta = vthetaMap.get(currentTime + shootLatancyMs.get());

        Rotation2d robotAngle = robotPose.toPose2d().getRotation();
        Translation2d robotRelOffset =
                ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
        Translation2d fieldRelOffset = robotRelOffset.rotateBy(robotAngle);

        double tVx = pVx - (pVtheta * fieldRelOffset.getY());
        double tVy = pVy - (pVtheta * fieldRelOffset.getX());
        Translation2d turretVel = new Translation2d(tVx, tVy);

        Translation2d virtTarget2d = target.toTranslation2d();
        double targetZ = target.getZ();
        double impactAngleRad = Math.toRadians(impactAngle.get());

        double hoodAngle = 0.0;
        double ballVelocity = 0.0;
        double distFromPivotToTarget = 0.0;
        double heightFromPivotToTarget = targetZ - fieldRelTurret.getZ();

        // Iteratively solve for flight time and target lead
        for (int i = 0; i < (int) calcIterations.get(); i++) {
            distFromPivotToTarget = turretPos2d.getDistance(virtTarget2d);
            Translation2d relVirtTarget = new Translation2d(distFromPivotToTarget, heightFromPivotToTarget);

            hoodAngle = findLaunchAngle(relVirtTarget, impactAngleRad, maxAngle);
            ballVelocity = calculateVel(hoodAngle, relVirtTarget);

            double v0x = ballVelocity * Math.cos(hoodAngle);
            double flightTime = (distFromPivotToTarget / v0x) * 1000;
            double totalTime = flightTime + shootLatancyMs.get();

            virtTarget2d = target.toTranslation2d().minus(turretVel.times(totalTime / 1000));
        }

        double turretAngle = calculateFieldRelativeTurret(robotPose.toPose2d(), virtTarget2d);

        ShotParametersAutoLogged rawShot = new ShotParametersAutoLogged();
        rawShot.hoodAngle = Math.PI / 2 - hoodAngle;
        rawShot.turretAngle = turretAngle;
        rawShot.ballVelocity = ballVelocity;

        // Apply hysteresis/tolerance check
        boolean acceptedNew = true;
        if (tolerance > 0.0 && lastAcceptedVirtTarget2d != null && lastAcceptedShot != null) {
            if (virtTarget2d.minus(lastAcceptedVirtTarget2d).getNorm() <= tolerance) {
                acceptedNew = false;
                lastAcceptedShot.turretAngle = rawShot.turretAngle;
            }
        }

        ShotParametersAutoLogged shotToUse = (acceptedNew || lastAcceptedShot == null) ? rawShot : lastAcceptedShot;
        if (acceptedNew || lastAcceptedShot == null) {
            lastAcceptedShot = rawShot;
            lastAcceptedVirtTarget2d = virtTarget2d;
        }

        // Apply air resistance compensation and RPM conversion
        shotToUse.ballVelocity *= 1 + (airResistanceMultiplier.get() * distFromPivotToTarget);
        shotToUse.wheelSpeed = shooterMPSToRPM(shotToUse.ballVelocity);

        return shotToUse;
    }

    /**
     * Calculates the turret angle required to aim at a target, accounting for the latency
     * between the current time and when the shot will be fired.
     *
     * @param robotPose      Current field-relative robot pose.
     * @param targetLocation Field-relative coordinates of the target.
     * @param currrentTime   The current timestamp in seconds.
     * @return The turret angle offset required for latency compensation in radians.
     */
    public double calculateLatancyOffsetTurretAngle(
            Pose2d robotPose, Translation2d targetLocation, double currrentTime) {
        double latancy = shootLatancyMs.get();

        Pose2d predictedRobotPose = new Pose2d(
                new Translation2d(
                        robotPose.getX() + vxMap.get(currrentTime + latancy) * latancy / 1000,
                        robotPose.getY() + vyMap.get(currrentTime + latancy) * latancy / 1000),
                new Rotation2d(
                        robotPose.getRotation().getRadians() + vthetaMap.get(currrentTime + latancy) * latancy / 1000));

        return calculateFieldRelativeTurret(predictedRobotPose, targetLocation);
    }

    private static double calculateFieldRelativeTurret(Pose2d robotPose, Translation2d targetLocation) {
        Transform2d turretOffset =
                new Transform2d(ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(), new Rotation2d());
        Pose2d turretFieldPose = robotPose.transformBy(turretOffset);

        Translation2d relativeTrans = targetLocation.minus(turretFieldPose.getTranslation());
        Rotation2d fieldRelativeAngle = new Rotation2d(relativeTrans.getX(), relativeTrans.getY());

        Rotation2d robotRelAngle = fieldRelativeAngle.minus(robotPose.getRotation());
        return MathUtil.inputModulus(robotRelAngle.getRadians() + Math.PI, -Math.PI, Math.PI);
    }

    public static double shooterMPSToRPM(double mps) {
        return (60.0 * mps) / ((5.0 / 6.0) * Math.PI * (2.0 * ShooterConstants.SHOOTER_BIG_WHEEL_RADIUS_METERS));
    }

    private static double findLaunchAngle(Translation2d trans, double alpha, double maxAngle) {
        double low = Math.PI / 2 - maxAngle;
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
