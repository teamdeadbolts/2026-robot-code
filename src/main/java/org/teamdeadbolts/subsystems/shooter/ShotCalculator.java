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
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.TimedExtrapolatingDoubleMap;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

/**
 * Calculates optimal shooter parameters (hood angle, wheel speed, turret angle)
 * to land a game piece in a target, accounting for robot motion, ball flight
 * time, 2D quadratic air resistance, and optimal impact angle to minimize RPM.
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

    private Translation2d lastAcceptedVirtTarget2d = null;
    private ShotParametersAutoLogged lastAcceptedShot = null;

    /* --- Tuning Parameters --- */
    private final SavedTunableNumber calcIterations = SavedTunableNumber.get("Tuning/Shooter/CalcIterations", 15);
    private final SavedTunableNumber minImpactAngle =
            SavedTunableNumber.get("Tuning/Shooter/MinImpactAngleDegrees", 10);
    private final SavedTunableNumber maxImpactAngle =
            SavedTunableNumber.get("Tuning/Shooter/MaxImpactAngleDegrees", 45);
    private final SavedTunableNumber shootLatancyMs = SavedTunableNumber.get("Tuning/Shooter/ShootLatancyMs", 0);
    private final SavedTunableNumber timeToKeepVel = SavedTunableNumber.get("Tuning/Shooter/TimeToKeepVelMs", 1000);
    private final SavedTunableNumber linerFilter = SavedTunableNumber.get("Tuning/Shooter/LinearFilter", 5);

    private final SavedTunableNumber dragCoefficient = SavedTunableNumber.get("Tuning/Shooter/DragCoefficient", 0.47);
    private final SavedTunableNumber airDensity = SavedTunableNumber.get("Tuning/Shooter/AirDensity", 1.1);
    private final SavedTunableNumber simStepSize = SavedTunableNumber.get("Tuning/Shooter/SimStepSizeSec", 0.02);

    private final SavedTunableNumber magnusCoefficient = SavedTunableNumber.get("Tuning/Shooter/MagnuxCoefficient", 0.015);

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

    public void updateVelocityState(final double timestamp, final ChassisSpeeds speeds) {
        final double smoothedVx = vxFilter.calculate(speeds.vxMetersPerSecond);
        final double smoothedVy = vyFilter.calculate(speeds.vyMetersPerSecond);
        final double smoothedVtheta = vthetaFilter.calculate(speeds.omegaRadiansPerSecond);

        vxMap.put(timestamp, smoothedVx);
        vyMap.put(timestamp, smoothedVy);
        vthetaMap.put(timestamp, smoothedVtheta);
    }

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
                new Rotation3d(
                        0, 0, predictedPose2d.getRotation().getRadians()));

        // 3. Calculate turret geometry based on the PREDICTED pose
        final Pose3d fieldRelTurret = predictedRobotPose.transformBy(ShooterConstants.SHOOTER_OFFSET);
        final Translation2d turretPos2d = fieldRelTurret.getTranslation().toTranslation2d();

        final Rotation2d robotAngle = predictedPose2d.getRotation();
        final Translation2d robotRelOffset =
                ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
        final Translation2d fieldRelOffset = robotRelOffset.rotateBy(robotAngle);

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
        double turretAngle = 0.0;
        final double heightFromPivotToTarget = targetZ - fieldRelTurret.getZ();

        // 4. Iteratively solve for flight time, target lead, and optimal impact angle (with Drag)
        for (int i = 0; i < calcIterations.get().intValue(); i++) {
            distFromPivotToTarget = turretPos2d.getDistance(virtTarget2d);
            final Translation2d relVirtTarget = new Translation2d(distFromPivotToTarget, heightFromPivotToTarget);

            double dx = Math.max(0.01, distFromPivotToTarget);
            double dy = heightFromPivotToTarget;
            if (i > 0) {
                dx = Math.max(0.01, dx - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(hoodAngle)));
                dy -= ShooterConstants.EXIT_RADIUS_METERS * Math.sin(hoodAngle);
            }


            final double k = dy / dx;
            final double idealAlphaRad = Math.atan(Math.sqrt(k * k + 1) - k);
            impactAngle = MathUtil.clamp(idealAlphaRad, minAlphaRad, maxAlphaRad);

            double[] simResults = new double[2]; // [0] = velocity, [1] = flightTime
            hoodAngle = findLaunchAngleWithDrag(relVirtTarget, impactAngle, maxAngle, simResults);

            ballVelocity = simResults[0];
            final double flightTimeSec = simResults[1];

            Rotation2d shootingDirection = virtTarget2d.minus(turretPos2d).getAngle();

            Translation2d ballVelVec = new Translation2d(
              ballVelocity * Math.cos(hoodAngle) * shootingDirection.getCos(),
              ballVelocity * Math.cos(hoodAngle) * shootingDirection.getSin()
            );

            Translation2d vRelative2d = ballVelVec.minus(turretVel);

            final Rotation2d compensatedTurretFieldAngle = vRelative2d.getAngle();
            turretAngle = MathUtil.angleModulus(compensatedTurretFieldAngle.minus(robotAngle).getRadians());

            virtTarget2d = target.toTranslation2d().minus(turretVel.times(flightTimeSec));
        }

        // final double turretAngle = calculateFieldRelativeTurret(predictedPose2d, virtTarget2d);

        final ShotParametersAutoLogged rawShot = new ShotParametersAutoLogged();
        rawShot.hoodAngle = Math.PI / 2 - hoodAngle;
        rawShot.turretAngle = turretAngle;
        rawShot.ballVelocity = ballVelocity;
        rawShot.rawLaunchAngleRad = hoodAngle;
        rawShot.impactAngle = impactAngle;
        rawShot.virtTarget = new Pose3d(virtTarget2d.getX(), virtTarget2d.getY(), targetZ, new Rotation3d());
        rawShot.wheelSpeed = shooterMPSToRPM(ballVelocity);

        Logger.processInputs("ShotCalc/RawShot", rawShot);

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

        rawShot.isPossible = ShooterConstants.SHOOTER_MAX_RPM >= shotToUse.wheelSpeed;

        return rawShot;
    }

    public static double shooterMPSToRPM(final double mps) {
        return ShooterConstants.SHOOTER_MPS_TO_RPM_MAP.get(mps);
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

    public double calculateLatancyOffsetTurretAngle(
            final Pose2d robotPose, final Translation2d targetLocation, final double currentTime) {
        final double latencyMs = shootLatancyMs.get();
        final double latencySec = latencyMs / 1000.0;
        final double predictedVx = vxMap.get(currentTime + latencySec);
        final double predictedVy = vyMap.get(currentTime + latencySec);
        final double predictedOmega = vthetaMap.get(currentTime + latencySec);

        final Pose2d predictedRobotPose = new Pose2d(
                new Translation2d(
                        robotPose.getX() + (predictedVx * latencySec), robotPose.getY() + (predictedVy * latencySec)),
                new Rotation2d(robotPose.getRotation().getRadians() + (predictedOmega * latencySec)));

        return calculateFieldRelativeTurret(predictedRobotPose, targetLocation);
    }

    /**
     * Simulates the trajectory of the ball using 2D quadratic drag and Euler integration.
     * Returns the exact interpolated Y height when the ball crosses the target X.
     */
    private double simulateY(double v0, double theta, Translation2d target, double[] outParams) {
        double x = ShooterConstants.EXIT_RADIUS_METERS * Math.cos(theta);
        double y = ShooterConstants.EXIT_RADIUS_METERS * Math.sin(theta);
        double vx = v0 * Math.cos(theta);
        double vy = v0 * Math.sin(theta);

        double t = 0;
        double dt = simStepSize.get();

        double area = Math.PI * Math.pow(ShooterConstants.BALL_RADIUS_METERS, 2);
        // k = (rho * Cd * A) / 2m
        double kDrag = 0.5 * airDensity.get() * dragCoefficient.get() * area / ShooterConstants.BALL_MASS_KG;
        double kMagnus = magnusCoefficient.get() * v0;
        while (x < target.getX() && t < 3.0) { // Safeties: 3 sec max
            double v = Math.sqrt(vx * vx + vy * vy);

            // Apply accelerations based on quadratic drag forces
            double axDrag = -kDrag * v * vx;
            double ayDrag = -kDrag * v * vy;

            double axMagnus = -kMagnus * vy;
            double ayMagnus = kMagnus * vx;

            vx += (axDrag + axMagnus) * dt;
            vy += (-G + ayDrag + ayMagnus) * dt;
            x += vx * dt;
            y += vy * dt;
            t += dt;
        }

        // Linearly interpolate for exact Y intersection and flight time at target X
        double xPrev = x - vx * dt;
        double yPrev = y - vy * dt;
        double dx = x - xPrev;

        double exactY = y;
        double exactT = t;

        if (dx > 0) {
            double frac = (target.getX() - xPrev) / dx;
            exactY = yPrev + frac * (y - yPrev);
            exactT = (t - dt) + frac * dt;
        }

        if (outParams != null && outParams.length >= 2) {
            outParams[0] = exactT;
            outParams[1] = Math.abs(Math.atan2(vy, vx)); // Store absolute impact angle magnitude
        }

        return exactY;
    }

    /** Binary searches to find the initial velocity required to hit the target (X,Y) given an angle */
    private double findRequiredVelocity(double theta, Translation2d target, double[] outParams) {
        double low = 0.0;
        double high = 10.5;
        int iterations = calcIterations.get().intValue();

        for (int i = 0; i < iterations; i++) {
            double mid = (low + high) / 2.0;
            double y = simulateY(mid, theta, target, outParams);
            if (y < target.getY()) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return (low + high) / 2.0;
    }

    private double findLaunchAngleWithDrag(
            Translation2d target, double desiredImpactAngle, double maxAngle, double[] outSimResults) {
        double lowTheta = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES);
        double highTheta = Math.PI / 2 - Math.toRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES);

        int iterations = calcIterations.get().intValue();
        double bestV0 = 0;

        for (int i = 0; i < iterations; i++) {
            double midTheta = (lowTheta + highTheta) / 2.0;
            double[] params = new double[2]; // [0] = flightTime, [1] = actualImpactAngle

            bestV0 = findRequiredVelocity(midTheta, target, params);

            if (params[1] > desiredImpactAngle) {
                highTheta = midTheta;
            } else {
                lowTheta = midTheta;
            }

            if (i == iterations - 1 && outSimResults != null && outSimResults.length >= 2) {
                outSimResults[0] = bestV0;
                outSimResults[1] = params[0]; // Flight time
            }
        }
        return (lowTheta + highTheta) / 2.0;
    }
}
