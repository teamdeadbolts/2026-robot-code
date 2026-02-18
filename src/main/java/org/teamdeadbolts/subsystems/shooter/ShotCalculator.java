/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShotCalculator {
    @AutoLog
    public static class ShotParameters {
        public double hoodAngle;
        public double turretAngle;
        public double wheelSpeed;
    }

    private static final double G = 9.81;

    private static final SavedLoggedNetworkNumber calcIterations =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/CalcIterations", 3);

    private static final SavedLoggedNetworkNumber impactAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ImpactAngleDegrees", 20);

    private static final SavedLoggedNetworkNumber slipCoe =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/SlipCoefficient", 1.0);

    public static ShotParametersAutoLogged calculateShot(Pose3d robotPose, Translation3d target) {
        Pose3d fieldRelTurret = robotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);

        Translation2d turretPos2d = fieldRelTurret.getTranslation().toTranslation2d();
        double distFromPivotToTarget = turretPos2d.getDistance(target.toTranslation2d());

        double heightFromPivotToTarget = target.getZ() - fieldRelTurret.getZ();

        Translation2d relativeTarget = new Translation2d(distFromPivotToTarget, heightFromPivotToTarget);

        double impactAngleRad = Math.toRadians(impactAngle.get());
        double hoodAngle = findLaunchAngle(relativeTarget, impactAngleRad);

        double wheelVel = calculateVel(hoodAngle, relativeTarget);

        double turretAngle = calculateFieldRelativeTurrent(target.toTranslation2d());

        ShotParametersAutoLogged shot = new ShotParametersAutoLogged();
        Logger.recordOutput("ShotCalc/PivToTarget", distFromPivotToTarget);
        Logger.recordOutput("ShotCalc/HeightFromPivot", heightFromPivotToTarget);
        Logger.recordOutput("ShotCalc/HoodAngle", Units.radiansToDegrees(hoodAngle));
        Logger.recordOutput("ShotCalc/Velocity", wheelVel);
        shot.hoodAngle = Math.PI / 2 - hoodAngle;
        shot.turretAngle = turretAngle;
        shot.wheelSpeed = shooterMPSToRPM(wheelVel);
        return shot;
    }

    private static double calculateFieldRelativeTurrent(Translation2d targetLocation) {
        Pose2d robotPose = RobotState.getInstance().getRobotPose().toPose2d();

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
