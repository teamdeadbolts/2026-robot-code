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
        public double flightTimeSec;

        public boolean isPossible;

        public Translation3d virtTarget;

        @Override
        public String toString() {
            return String.format(
                    "ShotParameters{hoodAngle=%.2f, turretAngle=%.2f, wheelSpeed=%.2f, flightTimeSec=%.2f, isPossible=%b}",
                    hoodAngle, turretAngle, wheelSpeed, flightTimeSec, isPossible);
        }
    }

    private Translation2d lastAcceptedVirtTarget2d = null;
    private ShotParametersAutoLogged lastAcceptedShot = null;

    private static final double G = 9.81;

    /* --- Tuning Parameters --- */
    private final SavedLoggedNetworkNumber calcInterations =
            SavedLoggedNetworkNumber.get("Tuning/ShotCalc/CalcInterations", 5);
    private final SavedLoggedNetworkNumber shootLatancyMs =
            SavedLoggedNetworkNumber.get("Tuning/ShotCalc/ShootLatancyMs", 0.0);
    private final SavedLoggedNetworkNumber timeToKeepVel =
            SavedLoggedNetworkNumber.get("Tuning/ShotCalc/TimeToKeepVel", 1000);
    private final SavedLoggedNetworkNumber velLinearFilter =
            SavedLoggedNetworkNumber.get("Tuning/ShotCalc/VelLinearFilter", 5);

    private final TimedExtrapolatingDoubleMap vxMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vyMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vthetaMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());

    private LinearFilter vxFilter = LinearFilter.movingAverage((int) velLinearFilter.get());
    private LinearFilter vyFilter = LinearFilter.movingAverage((int) velLinearFilter.get());
    private LinearFilter vthetaFilter = LinearFilter.movingAverage((int) velLinearFilter.get());

    public ShotCalculator() {
        velLinearFilter.addRefreshable(this);
    }

    @Override
    public void refresh() {
        vxFilter = LinearFilter.movingAverage((int) velLinearFilter.get());
        vyFilter = LinearFilter.movingAverage((int) velLinearFilter.get());
        vthetaFilter = LinearFilter.movingAverage((int) velLinearFilter.get());
    }

    /**
     * Updates the velocity state with the given timestamp and chassis speeds.
     * @param timestamp The current timestamp.
     * @param speeds The chassis speeds to update from.
     */
    public void updateVelocityState(double timestamp, ChassisSpeeds speeds) {
        double filteredVx = vxFilter.calculate(speeds.vxMetersPerSecond);
        double filteredVy = vyFilter.calculate(speeds.vyMetersPerSecond);
        double filteredVtheta = vthetaFilter.calculate(speeds.omegaRadiansPerSecond);
        vxMap.put(timestamp, filteredVx);
        vyMap.put(timestamp, filteredVy);
        vthetaMap.put(timestamp, filteredVtheta);
    }

    /**
     * Calculates shot parameters using distance-based interpolation maps.
     * Uses iterative solving to compensate for target movement during flight time.
     *
     * @param robotPose Current field-relative robot pose.
     * @param target Field-relative 3D target coordinates.
     * @param currentTime Current timestamp.
     * @param tolerance Distance threshold for updating target aim.
     * @return Calculated shot parameters.
     */
    public ShotParametersAutoLogged calculateShot(
            Pose3d robotPose, Translation3d target, double currentTime, double tolerance) {
        double latencySec = shootLatancyMs.get() / 1000.0;
        double pVx = vxMap.get(currentTime + latencySec);
        double pVy = vyMap.get(currentTime + latencySec);
        double pVtheta = vthetaMap.get(currentTime + latencySec);

        Pose2d currPose = robotPose.toPose2d();
        Pose2d predictedPose = new Pose2d(
                currPose.getX() + pVx * latencySec,
                currPose.getY() + pVy * latencySec,
                new Rotation2d(currPose.getRotation().getRadians() + pVtheta * latencySec));

        Pose3d predictedRobotPose = new Pose3d(
                predictedPose.getX(),
                predictedPose.getY(),
                robotPose.getZ(),
                new Rotation3d(0, 0, predictedPose.getRotation().getRadians()));

        Pose3d fieldRelTurret = predictedRobotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);
        Translation2d turretPose2d = fieldRelTurret.getTranslation().toTranslation2d();

        Rotation2d robotAngle = predictedPose.getRotation();
        Translation2d robotRelOffset =
                ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
        Translation2d fieldRelOffset = robotRelOffset.rotateBy(robotAngle);

        double tVx = pVx - (pVtheta * fieldRelOffset.getY());
        double tVy = pVy + (pVtheta * fieldRelOffset.getX());
        Translation2d turretVel = new Translation2d(tVx, tVy);

        Translation2d virtTarget2d = target.toTranslation2d();

        double distFromPivotToTarget = 0.0;
        double flightTimeSec = 0.0;

        for (int i = 0; i < calcInterations.get(); i++) {
            distFromPivotToTarget = turretPose2d.getDistance(virtTarget2d);
            flightTimeSec = ShooterConstants.DISTNACE_TO_TOF.get(distFromPivotToTarget);
            virtTarget2d = target.toTranslation2d().minus(turretVel.times(flightTimeSec));
        }

        distFromPivotToTarget = turretPose2d.getDistance(virtTarget2d);

        double hoodAngle = ShooterConstants.DISTANCE_TO_HOOD_ANGLE.get(distFromPivotToTarget);
        double wheelSpeed = ShooterConstants.DISTANCE_TO_RPM.get(distFromPivotToTarget);
        double turretAngle = calculateFieldRelativeTurret(predictedPose, virtTarget2d);

        ShotParametersAutoLogged rawShot = new ShotParametersAutoLogged();
        rawShot.hoodAngle = hoodAngle;
        rawShot.turretAngle = turretAngle;
        rawShot.wheelSpeed = wheelSpeed;
        rawShot.flightTimeSec = flightTimeSec;

        Logger.processInputs("ShotCalc/RawShot", rawShot);

        boolean acceptedNew = true;
        if (tolerance > 0.0 && lastAcceptedVirtTarget2d != null && lastAcceptedShot != null) {
            if (virtTarget2d.minus(lastAcceptedVirtTarget2d).getNorm() <= tolerance) {
                acceptedNew = false;
                lastAcceptedShot.turretAngle = rawShot.turretAngle;
            }
        }

        ShotParametersAutoLogged shotToUse = acceptedNew ? rawShot : lastAcceptedShot;

        if (acceptedNew) {
            lastAcceptedVirtTarget2d = virtTarget2d;
            lastAcceptedShot = rawShot;
        }

        shotToUse.isPossible = shotToUse.wheelSpeed <= ShooterConstants.SHOOTER_MAX_RPM;

        Logger.processInputs("ShotCalc/RealShot", shotToUse);

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

        double latencySec = shootLatancyMs.get() / 1000.0;

        double predictedVx = vxMap.get(currentTime + latencySec);
        double predictedVy = vyMap.get(currentTime + latencySec);
        double predictedOmega = vthetaMap.get(currentTime + latencySec);

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
}
