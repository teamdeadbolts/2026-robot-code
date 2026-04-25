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
        public double turretVelocity;
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

    private Translation2d lastAcceptedVirtTarget2d = null;
    private ShotParametersAutoLogged lastAcceptedShot = null;

    /* --- Tuning Parameters --- */
    private final SavedTunableNumber calcIterations = SavedTunableNumber.get("Tuning/Shooter/CalcIterations", 15);
    private final SavedTunableNumber minImpactAngle =
            SavedTunableNumber.get("Tuning/Shooter/MinImpactAngleDegrees", 10);
    private final SavedTunableNumber maxImpactAngle =
            SavedTunableNumber.get("Tuning/Shooter/MaxImpactAngleDegrees", 45);
    private final SavedTunableNumber impactBias = SavedTunableNumber.get("Tuning/Shooter/LowImpactBias", 0);
    private final SavedTunableNumber shootLatancyMs = SavedTunableNumber.get("Tuning/Shooter/ShootLatancyMs", 0);
    private final SavedTunableNumber timeToKeepVel = SavedTunableNumber.get("Tuning/Shooter/TimeToKeepVelMs", 1000);
    private final SavedTunableNumber linerFilter = SavedTunableNumber.get("Tuning/Shooter/LinearFilter", 5);

    /* --- Air Resistance & Ballistics Tuning Parameters --- */
    private final SavedTunableNumber dragCoefficient =
            SavedTunableNumber.get("Tuning/Shooter/DragCoefficient", 0.47); // Rough sphere default
    private final SavedTunableNumber airDensity =
            SavedTunableNumber.get("Tuning/Shooter/AirDensity", 1.1); // Sea level kg/m^3
    private final SavedTunableNumber simStepSize =
            SavedTunableNumber.get("Tuning/Shooter/SimStepSizeSec", 0.02); // 20ms integration step

    private final TimedExtrapolatingDoubleMap vxMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vyMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap vthetaMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());

    private final TimedExtrapolatingDoubleMap axMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap ayMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());
    private final TimedExtrapolatingDoubleMap athetaMap = new TimedExtrapolatingDoubleMap(timeToKeepVel.get());

    private double lastTime = 0.0;
    private double lastVx = 0.0;
    private double lastVy = 0.0;
    private double lastVtheta = 0.0;

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

        double dt = (timestamp - lastTime) / 1000;
        if (dt > 0.0 && dt < 0.1) {
            axMap.put(timestamp, (smoothedVx - lastVx) / dt);
            ayMap.put(timestamp, (smoothedVy - lastVy) / dt);
            athetaMap.put(timestamp, (smoothedVtheta - lastVtheta) / dt);
        } else {
            axMap.put(timestamp, 0.0);
            ayMap.put(timestamp, 0.0);
            athetaMap.put(timestamp, 0.0);
        }

        lastTime = timestamp;
        lastVx = smoothedVx;
        lastVy = smoothedVy;
        lastVtheta = smoothedVtheta;
    }

    public ShotParametersAutoLogged calculateShot(
            final Pose3d robotPose,
            final Translation3d target,
            final double currentTime,
            final double tolerance,
            final double maxAngle) {

        double latency = shootLatancyMs.get();
        final double latencySec = latency / 1000.0;
        double pVx = vxMap.get(currentTime + latency);
        double pVy = vyMap.get(currentTime + latency);
        double pVtheta = vthetaMap.get(currentTime + latency);

        double pAx = axMap.get(currentTime + latency);
        double pAy = ayMap.get(currentTime + latency);
        double pAtheta = athetaMap.get(currentTime + latency);

        Pose2d currentPose2d = robotPose.toPose2d();

        double dx = (pVx * latencySec) + (pAx * latencySec * latencySec / 2.0);
        double dy = (pVy * latencySec) + (pAy * latencySec * latencySec / 2.0);
        double dtheta = (pVtheta * latencySec) + (pAtheta * latencySec * latencySec / 2.0);

        Pose2d predictedPose2d = new Pose2d(
                currentPose2d.getX() + dx,
                currentPose2d.getY() + dy,
                new Rotation2d(currentPose2d.getRotation().getRadians() + dtheta));

        final Pose3d predictedRobotPose = new Pose3d(
                predictedPose2d.getX(),
                predictedPose2d.getY(),
                robotPose.getZ(),
                new edu.wpi.first.math.geometry.Rotation3d(
                        0, 0, predictedPose2d.getRotation().getRadians()));

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
        final double heightFromPivotToTarget = targetZ - fieldRelTurret.getZ();
        double fieldRelativeTurretOmega = 0.0;

        for (int i = 0; i < calcIterations.get().intValue(); i++) {
            distFromPivotToTarget = turretPos2d.getDistance(virtTarget2d);
            Translation2d relVirtTarget = new Translation2d(distFromPivotToTarget, heightFromPivotToTarget);

            // Resuse dx and dy vars
            dx = Math.max(0.01, distFromPivotToTarget);
            dy = heightFromPivotToTarget;
            if (i > 0) {
                dx = Math.max(0.01, dx - (ShooterConstants.EXIT_RADIUS_METERS * Math.cos(hoodAngle)));
                dy -= ShooterConstants.EXIT_RADIUS_METERS * Math.sin(hoodAngle);
            }

            double k = dy / dx;
            double bestAngle = Math.atan(Math.sqrt(k * k + 1) - k);
            double balanced = (bestAngle * (1.0 - impactBias.get())) + (minAlphaRad * impactBias.get());
            impactAngle = MathUtil.clamp(balanced, minAlphaRad, maxAlphaRad);

            double[] simResults = new double[2];
            hoodAngle = findLaunchAngleWithDrag(relVirtTarget, impactAngle, maxAngle, simResults);
            ballVelocity = simResults[0];
            double flightTime = simResults[1];

            Translation2d turretToTarget = virtTarget2d.minus(turretPos2d);
            double dSq = Math.max(0.01, Math.pow(turretToTarget.getX(), 2) + Math.pow(turretToTarget.getY(), 2));

            double relVx = -turretVel.getX();
            double relVy = -turretVel.getY();

            fieldRelativeTurretOmega = (turretToTarget.getX() * relVy - turretToTarget.getY() * relVx) / dSq;

            double whipSpeed = fieldRelativeTurretOmega * ShooterConstants.EXIT_RADIUS_METERS * Math.cos(hoodAngle);

            double norm = Math.sqrt(dSq);
            Translation2d targentDir = new Translation2d(-turretToTarget.getY() / norm, turretToTarget.getX() / norm);
            Translation2d whipVel = targentDir.times(whipSpeed);

            Translation2d totalVelDistortion = turretVel.plus(whipVel);
            virtTarget2d = target.toTranslation2d().minus(totalVelDistortion.times(flightTime));
        }

        final double turretAngle = calculateFieldRelativeTurret(predictedPose2d, virtTarget2d);

        final ShotParametersAutoLogged rawShot = new ShotParametersAutoLogged();
        rawShot.hoodAngle = Math.PI / 2 - hoodAngle;
        rawShot.turretAngle = turretAngle;
        rawShot.turretVelocity = fieldRelativeTurretOmega - pVtheta;
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

        rawShot.isPossible = 5000.0 >= shotToUse.wheelSpeed;

        return rawShot;
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

    // --- NEW PHYSICS SIMULATION METHODS --- //

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

        double area = Math.PI * Math.pow(Units.inchesToMeters(6) / 2.0, 2);
        // k = (rho * Cd * A) / 2m
        double k = 0.5 * airDensity.get() * dragCoefficient.get() * area / 0.226;

        while (x < target.getX() && y > -2.0 && t < 3.0) {
            double v = Math.sqrt(vx * vx + vy * vy);

            double ax = -k * v * vx;
            double ay = -G - k * v * vy;

            vx += ax * dt;
            vy += ay * dt;
            x += vx * dt;
            y += vy * dt;
            t += dt;
        }

        double xPrev = x - vx * dt;
        double yPrev = y - vy * dt;
        double dx = x - xPrev;

        double exact_y = y;
        double exact_t = t;

        if (dx > 0) {
            double frac = (target.getX() - xPrev) / dx;
            exact_y = yPrev + frac * (y - yPrev);
            exact_t = (t - dt) + frac * dt;
        }

        if (outParams != null && outParams.length >= 2) {
            outParams[0] = exact_t;
            outParams[1] = Math.abs(Math.atan2(vy, vx));
        }

        return exact_y;
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
