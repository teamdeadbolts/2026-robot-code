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
        public double turrentAngle;
        public double wheelSpeed;
    }

    private static final double G = 9.81;
    private static final SavedLoggedNetworkNumber heightAbove =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HeightAboveTarget", 0);

    public static ShotParametersAutoLogged calculateShot(Pose3d robotPose, Translation3d target) {
        Pose3d turrentFieldPose = robotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);

        double dx = Math.hypot(target.getX() - turrentFieldPose.getX(), target.getY() - turrentFieldPose.getY());

        double h = (target.getZ() + heightAbove.get()) - turrentFieldPose.getZ();

        double hoodAngle = Math.atan2(2 * h, dx);

        double numerator = G * dx;
        double denominator = Math.sin(hoodAngle) * Math.cos(hoodAngle);
        Logger.recordOutput("ShooterCalc/Numer", numerator);
        Logger.recordOutput("ShooterCalc/Denom", denominator);

        double velocity = Math.sqrt(numerator / denominator);

        ShotParametersAutoLogged result = new ShotParametersAutoLogged();
        result.hoodAngle = MathUtil.clamp(
                Math.PI / 2 - hoodAngle,
                Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES),
                Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES));
        result.turrentAngle = calculateFieldRelativeTurrent(new Translation2d(target.getX(), target.getY()));
        double wheelSpeed = shooterMPSToRPM(velocity);
        Logger.recordOutput("ShooterCalc/TargetVelMPM", velocity);
        result.wheelSpeed = wheelSpeed;

        return result;
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

    private static double shooterMPSToRPM(double mps) {
        return (60.0 * mps)
                / (Math.PI
                        * (ShooterConstants.SHOOTER_BIG_WHEEL_RADIUS_METERS
                                + ShooterConstants.SHOOTER_SMALL_WHEEL_RADIUS_METERS));
    }
}
