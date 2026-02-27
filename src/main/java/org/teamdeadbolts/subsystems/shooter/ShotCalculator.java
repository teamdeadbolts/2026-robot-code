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
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.ExtrapolatingDoubleMap;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShotCalculator {
    @AutoLog
    public static class ShotParameters {
        public double hoodAngle;
        public double turretAngle;
        public double wheelSpeed;
        public double ballVelocity;

        @Override
        public String toString() {
            return "ShotParameters{" + "hoodAngle=" + hoodAngle + ", turretAngle=" + turretAngle + ", wheelSpeed="
                    + wheelSpeed + ", ballVelocity=" + ballVelocity + "}";
        }
    }

    private static final double G = 9.81;

    private static final SavedLoggedNetworkNumber calcIterations =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/CalcIterations", 3);

    private static final SavedLoggedNetworkNumber impactAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ImpactAngleDegrees", 20);

    private static final SavedLoggedNetworkNumber shootLatancyMs =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShootLatancyMs", 100);
    private static final SavedLoggedNetworkNumber timeToKeepVel =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TimeToKeepVelMs", 1000);

    private final ExtrapolatingDoubleMap vxMap = new ExtrapolatingDoubleMap(timeToKeepVel.get());
    private final ExtrapolatingDoubleMap vyMap = new ExtrapolatingDoubleMap(timeToKeepVel.get());
    private final ExtrapolatingDoubleMap vthetaMap = new ExtrapolatingDoubleMap(timeToKeepVel.get());

    public ShotCalculator() {}
    ;

    public void updateVelocityState(double timestamp, ChassisSpeeds speeds) {
        vxMap.put(timestamp, speeds.vxMetersPerSecond);
        vyMap.put(timestamp, speeds.vyMetersPerSecond);
        vthetaMap.put(timestamp, speeds.omegaRadiansPerSecond);
    }

    public ShotParametersAutoLogged calculateShot(Pose3d robotPose, Translation3d target, double currentTime) {
        Pose3d fieldRelTurret = robotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);
        Translation2d turretPos2d = fieldRelTurret.getTranslation().toTranslation2d();

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
        for (int i = 0; i < calcIterations.get(); i++) {
            distFromPivotToTarget = turretPos2d.getDistance(virtTarget2d);
            Translation2d relVirtTarget = new Translation2d(distFromPivotToTarget, heightFromPivotToTarget);

            hoodAngle = findLaunchAngle(relVirtTarget, impactAngleRad);
            ballVelocity = calculateVel(hoodAngle, relVirtTarget);

            double v0x = ballVelocity * Math.cos(hoodAngle);
            double flightTime = (distFromPivotToTarget / v0x) * 1000;
            double totalTime = flightTime + shootLatancyMs.get();

            virtTarget2d = target.toTranslation2d().minus(turretVel.times(totalTime / 1000));
        }

        double turretAngle = calculateFieldRelativeTurrent(robotPose.toPose2d(), virtTarget2d);

        ShotParametersAutoLogged shot = new ShotParametersAutoLogged();
        Logger.recordOutput("ShotCalc/VirtualTarget", new Pose2d(virtTarget2d, new Rotation2d()));
        Logger.recordOutput("ShotCalc/TurretVel", turretVel);
        Logger.recordOutput("ShotCalc/PivToVirtualTarget", distFromPivotToTarget);
        Logger.recordOutput("ShotCalc/HeightFromPivot", heightFromPivotToTarget);
        Logger.recordOutput("ShotCalc/HoodAngle", Units.radiansToDegrees(hoodAngle));
        Logger.recordOutput("ShotCalc/Velocity", ballVelocity);
        shot.hoodAngle = Math.PI / 2 - hoodAngle;
        shot.turretAngle = turretAngle;
        shot.wheelSpeed = shooterMPSToRPM(ballVelocity);
        shot.ballVelocity = ballVelocity;

        return shot;
    }

    public double calculateLatancyOffsetTurrentAngle(
            Pose2d robotPose, Translation2d targetLocation, double currrentTime) {
        double latancy = shootLatancyMs.get();

        Pose2d predictedRobotPose = new Pose2d(
                new Translation2d(
                        robotPose.getX() + vxMap.get(currrentTime + latancy) * latancy / 1000,
                        robotPose.getY() + vyMap.get(currrentTime + latancy) * latancy / 1000),
                new Rotation2d(
                        robotPose.getRotation().getRadians() + vthetaMap.get(currrentTime + latancy) * latancy / 1000));

        return calculateFieldRelativeTurrent(predictedRobotPose, targetLocation);
    }

    private static double calculateFieldRelativeTurrent(Pose2d robotPose, Translation2d targetLocation) {

        Transform2d turretOffset =
                new Transform2d(ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(), new Rotation2d());
        Pose2d turretFieldPose = robotPose.transformBy(turretOffset);

        Translation2d relativeTrans = targetLocation.minus(turretFieldPose.getTranslation());
        Rotation2d fieldRelativeAngle = new Rotation2d(relativeTrans.getX(), relativeTrans.getY());

        Rotation2d robotRelAngle = fieldRelativeAngle.minus(robotPose.getRotation());
        return MathUtil.inputModulus(robotRelAngle.getRadians() + Math.PI, -Math.PI, Math.PI);
    }

    public static double shooterMPSToRPM(double mps) {
        double compressionOffsetMeters = Units.inchesToMeters((6.0 - 4.986) / 2.0);

        double effectiveBigRadius = ShooterConstants.SHOOTER_BIG_WHEEL_RADIUS_METERS - compressionOffsetMeters;
        double effectiveSmallRadius = ShooterConstants.SHOOTER_SMALL_WHEEL_RADIUS_METERS - compressionOffsetMeters;

        // return (60.0 * mps * slipCoe.get()) / (Math.PI * (effectiveBigRadius + effectiveSmallRadius));
        return (60.0 * mps) / ((5.0 / 6.0) * Math.PI * (2.0 * ShooterConstants.SHOOTER_BIG_WHEEL_RADIUS_METERS));
    }

    private static double findLaunchAngle(Translation2d trans, double alpha) {
        double low = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES);
        double high = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES);

        for (int i = 0; i < calcIterations.get(); i++) {
            double mid = (low + high) / 2;
            if (evaluateError(mid, trans, alpha) > 0) {
                high = mid;
            } else {
                low = mid;
            }
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

        double cos = Math.cos(theta);

        double denom = 2 * Math.pow(cos, 2) * (dx * Math.tan(theta) - dy);
        return Math.sqrt((G * Math.pow(dx, 2)) / denom);
    }
}
