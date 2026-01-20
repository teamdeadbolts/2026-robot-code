/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.constants.ShooterConstants.ExitOffset;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {
    public enum State {
        OFF,
        SPINUP,
        SHOOT,
        PASS_LEFT,
        PASS_RIGHT;
    }

    @AutoLogOutput
    private State targetState = State.OFF;

    private TalonFX turretMotor = new TalonFX(ShooterConstants.SHOOTER_TURRET_MOTOR_CAN_ID);
    private TalonFX hoodMotor = new TalonFX(ShooterConstants.SHOOTER_HOOD_MOTOR_CAN_ID);
    private TalonFX wheelMotor = new TalonFX(ShooterConstants.SHOOTER_WHEEL_MOTOR_CAN_ID);

    private CANcoder absEncoder = new CANcoder(ShooterConstants.SHOOTER_ABS_ENCODER_CAN_ID);

    private PIDController hoodController = new PIDController(0.0, 0.0, 0.0);
    private PIDController turretController = new PIDController(0.0, 0.0, 0.0);
    private PIDController wheelController = new PIDController(0.0, 0.0, 0.0);

    private SavedLoggedNetworkNumber hoodControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kP", 0.1);
    private SavedLoggedNetworkNumber hoodControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kI", 0.0);
    private SavedLoggedNetworkNumber hoodControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kD", 0.0);

    private SavedLoggedNetworkNumber turretControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kP", 0.1);
    private SavedLoggedNetworkNumber turretControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kI", 0.0);
    private SavedLoggedNetworkNumber turretControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kD", 0.0);

    private SavedLoggedNetworkNumber wheelControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kP", 0.1);
    private SavedLoggedNetworkNumber wheelControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kI", 0.0);
    private SavedLoggedNetworkNumber wheelControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kD", 0.0);

    private SavedLoggedNetworkNumber shooterWheelSpinupSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterWheelSpinupSpeed", 5000.0); // RPM

    private SavedLoggedNetworkNumber hoodStepSize =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodStepSize", 0.5); // Degrees
    private SavedLoggedNetworkNumber hoodInterationsPerStep =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodIterationsPerStep", 5); // Iterations

    private static final double kG = 9.81; // m/s^2
    private double targetWheelSpeed;
    private double currentWheelSpeed;

    public ShooterSubsystem() {
        ConfigManager.getInstance().onReady(this::reconfigure);
    }

    private void reconfigure() {
        hoodController.setPID(hoodControllerP.get(), hoodControllerI.get(), hoodControllerD.get());
        turretController.setPID(turretControllerP.get(), turretControllerI.get(), turretControllerD.get());
        wheelController.setPID(wheelControllerP.get(), wheelControllerI.get(), wheelControllerD.get());

        ShooterConstants.init();
        hoodMotor.getConfigurator().apply(ShooterConstants.SHOOTER_HOOD_MOTOR_CONFIG);
        turretMotor.getConfigurator().apply(ShooterConstants.SHOOTER_TURRET_MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(ShooterConstants.SHOOTER_WHEEL_MOTOR_CONFIG);
        absEncoder.getConfigurator().apply(ShooterConstants.SHOOTER_ABS_ENCODER_CONFIG);
    }

    public void setState(State newState) {
        targetState = newState;
    }

    public State getState() {
        return targetState;
    }

    public double getRPMError() {
        return targetWheelSpeed - currentWheelSpeed;
    }

    @Override
    public void periodic() {
        double currentHoodAngle = absEncoder.getPosition().getValueAsDouble();
        double targetHoodAngle = currentHoodAngle;

        double currentTurrentPosition = turretMotor.getPosition().getValueAsDouble();
        double targetTurretPosition = currentTurrentPosition;

        currentWheelSpeed = wheelMotor.getVelocity().getValueAsDouble();
        // targetWheelSpeed = currentWheelSpeed;

        Pose2d robotPose = RobotState.getInstance().getRobotPose().toPose2d();
        ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeRobotVelocities();

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        switch (targetState) {
            case OFF:
                targetHoodAngle = ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES;
                targetWheelSpeed = 0;
                break;
            case PASS_LEFT:
            case PASS_RIGHT:
                Pose3d targetPose;

                if (alliance == Alliance.Red) {
                    targetPose = targetState == State.PASS_LEFT
                            ? ShooterConstants.PASS_LEFT_POSE_RED
                            : ShooterConstants.PASS_RIGHT_POSE_RED;
                } else {
                    targetPose = targetState == State.PASS_LEFT
                            ? ShooterConstants.PASS_LEFT_POSE_BLUE
                            : ShooterConstants.PASS_RIGHT_POSE_BLUE;
                }

                ShooterAimingParameters aimingParams =
                        calculateAimingParameters(targetPose, robotPose, robotSpeeds, true);
                targetHoodAngle = aimingParams.hoodAngle;
                targetTurretPosition = aimingParams.turrentAngle;
                targetWheelSpeed = aimingParams.wheelSpeed;
                break;
            case SHOOT:
                Pose3d shootTargetPose =
                        (alliance == Alliance.Red) ? ShooterConstants.SHOOT_POSE_RED : ShooterConstants.SHOOT_POSE_BLUE;
                ShooterAimingParameters shootAimingParams =
                        calculateAimingParameters(shootTargetPose, robotPose, robotSpeeds, false);
                targetHoodAngle = shootAimingParams.hoodAngle;
                targetTurretPosition = shootAimingParams.turrentAngle;
                targetWheelSpeed = shootAimingParams.wheelSpeed;
                break;
            case SPINUP:
                targetWheelSpeed = shooterWheelSpinupSpeed.get();
                targetHoodAngle = ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES;
                break;
        }

        double hoodOutput = hoodController.calculate(currentHoodAngle, targetHoodAngle);
        hoodMotor.setVoltage(hoodOutput);

        double wheelOutput = wheelController.calculate(currentWheelSpeed, targetWheelSpeed);
        wheelMotor.setVoltage(wheelOutput);

        targetTurretPosition = calculateTurrentSetpoint(currentTurrentPosition, Math.toRadians(targetTurretPosition));
        double turretOutput = turretController.calculate(currentTurrentPosition, targetTurretPosition);
        turretMotor.setVoltage(turretOutput);

        Logger.recordOutput("ShooterSubsystem/TargetHoodAngle", targetHoodAngle);
        Logger.recordOutput("ShooterSubsystem/CurrentHoodAngle", currentHoodAngle);
        Logger.recordOutput("ShooterSubsystem/TargetTurretPosition", targetTurretPosition);
        Logger.recordOutput("ShooterSubsystem/CurrentTurretPosition", currentTurrentPosition);
        Logger.recordOutput("ShooterSubsystem/TargetWheelSpeed", targetWheelSpeed);
        Logger.recordOutput("ShooterSubsystem/CurrentWheelSpeed", currentWheelSpeed);
    }

    private double calculateTurrentSetpoint(double currentAngle, double targetAngle) {
        double error = ((targetAngle - currentAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        if (error < 0) error += 2 * Math.PI;
        error -= Math.PI;

        double shortestPath = currentAngle + error;

        if (shortestPath > Math.toRadians(ShooterConstants.TURRENT_MAX_POSITION_DEGREES)) {
            return shortestPath - 2 * Math.PI;
        } else if (shortestPath < Math.toRadians(ShooterConstants.TURRENT_MIN_POSITION_DEGREES)) {
            return shortestPath + 2 * Math.PI;
        }
        return shortestPath;
    }

    private record ShooterAimingParameters(double hoodAngle, double turrentAngle, double wheelSpeed) {}

    public ShooterAimingParameters calculateAimingParameters(
            Pose3d target, Pose2d current, ChassisSpeeds robotSpeedsFieldRelative, boolean passing) {
        final double robotYaw = current.getRotation().getRadians();
        // Robot center position in field
        final double robotX = current.getX();
        final double robotY = current.getY();
        // Robot->turret pivot translation in robot frame
        Translation3d turretOffsetRobot = ShooterConstants.SHOOTER_TO_TURRENT.getTranslation();
        // Rotate turret pivot XY offset into field frame using robot yaw
        Translation2d turretOffsetFieldXY = new Translation2d(turretOffsetRobot.getX(), turretOffsetRobot.getY())
                .rotateBy(new Rotation2d(robotYaw));
        // Turret pivot position in field
        Translation3d turretPivotField = new Translation3d(
                robotX + turretOffsetFieldXY.getX(), robotY + turretOffsetFieldXY.getY(), turretOffsetRobot.getZ());
        // Target position in field
        Translation3d targetField = target.getTranslation();
        // Robot field velocity
        final double vRobotX = robotSpeedsFieldRelative.vxMetersPerSecond;
        final double vRobotY = robotSpeedsFieldRelative.vyMetersPerSecond;
        ShooterAimingParameters best = null;
        for (double hoodRad = Math.toRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES);
                hoodRad <= Math.toRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES) + 1e-12;
                hoodRad += Math.toRadians(hoodStepSize.get())) {
            // Start turret yaw guess aimed directly at target from pivot (field frame).
            double turretFieldYaw = Math.atan2(
                    targetField.getY() - turretPivotField.getY(), targetField.getX() - turretPivotField.getX());
            ShooterAimingParameters cand = null;
            // Iterate to account for hood-dependent exit shifting with turret yaw
            for (int iter = 0; iter < hoodInterationsPerStep.get(); iter++) {
                Translation3d exitOffsetTurret = exitOffsetTurretFrame(hoodRad);
                // Rotate exit offset XY by turret FIELD yaw (because turret frame spins in the field)
                Translation2d exitOffsetFieldXY = new Translation2d(exitOffsetTurret.getX(), exitOffsetTurret.getY())
                        .rotateBy(new Rotation2d(turretFieldYaw));
                Translation3d exitField = turretPivotField.plus(
                        new Translation3d(exitOffsetFieldXY.getX(), exitOffsetFieldXY.getY(), exitOffsetTurret.getZ()));
                // Displacement from exit -> target in FIELD
                Translation3d d = targetField.minus(exitField);
                double dx = d.getX();
                double dy = d.getY();
                double dz = d.getZ();
                double range = Math.hypot(dx, dy);
                if (range < 1e-4) {
                    // Too close/degenerate, skip this hood
                    cand = null;
                    break;
                }
                // Solve required FIELD speed magnitude for this hood angle:
                // v = sqrt(g*x^2 / (2*cos^2(theta)*(x*tan(theta)-y)))
                double cos = Math.cos(hoodRad);
                double sin = Math.sin(hoodRad);
                double tan = Math.tan(hoodRad);
                double denom = 2.0 * cos * cos * (range * tan - dz);
                if (denom <= 0.0) {
                    cand = null;
                    break; // no real ballistic solution at this hood angle
                }
                double vField = Math.sqrt((kG * range * range) / denom);
                if (!Double.isFinite(vField) || vField <= 0.0) {
                    cand = null;
                    break;
                }
                // Components (FIELD)
                double vHoriz = vField * cos;
                double vVert = vField * sin;
                // Flight time based on horizontal component
                double t = range / vHoriz;
                if (!Double.isFinite(t) || t <= 0.0) {
                    cand = null;
                    break;
                }
                // Required horizontal FIELD velocity vector (must match dx,dy over time t)
                double vReqFieldX = dx / t;
                double vReqFieldY = dy / t;
                // Convert to required RELATIVE horizontal velocity (relative to robot)
                double vRelX = vReqFieldX - vRobotX;
                double vRelY = vReqFieldY - vRobotY;
                double vRelHoriz = Math.hypot(vRelX, vRelY);
                double vRelTotal = Math.hypot(vRelHoriz, vVert);
                // Turret should aim along RELATIVE horizontal velocity direction (field frame)
                turretFieldYaw = Math.atan2(vRelY, vRelX);
                // Convert required relative exit speed to wheel omega
                double wheelOmega = exitSpeedToWheelOmega(vRelTotal);
                wheelOmega = MathUtil.clamp(wheelOmega, 0.0, ShooterConstants.WHEEL_MAX_OMEGA_RAD_PER_SEC);
                cand = new ShooterAimingParameters(hoodRad, turretFieldYaw, wheelOmega);
            }
            if (cand == null) continue;
            best = pickBetter(best, cand, passing);
        }
        if (best == null) {
            // No solution within hood limits. Return a sane fallback:
            double fallbackTurretFieldYaw = Math.atan2(
                    targetField.getY() - turretPivotField.getY(), targetField.getX() - turretPivotField.getX());
            double turretAngleRobot = MathUtil.angleModulus(fallbackTurretFieldYaw - robotYaw);
            return new ShooterAimingParameters(
                    ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES,
                    turretAngleRobot,
                    ShooterConstants.WHEEL_MAX_OMEGA_RAD_PER_SEC);
        }
        // Convert turret FIELD yaw -> robot-relative turret angle
        double turretAngleRobot = MathUtil.angleModulus(best.turrentAngle() - robotYaw);
        return new ShooterAimingParameters(best.hoodAngle(), turretAngleRobot, best.wheelSpeed());
    }

    private static ShooterAimingParameters pickBetter(
            ShooterAimingParameters best, ShooterAimingParameters cand, boolean passing) {
        if (best == null) return cand;
        if (passing) {
            if (cand.hoodAngle() > best.hoodAngle() + 1e-9) return cand;
            if (Math.abs(cand.hoodAngle() - best.hoodAngle()) < 1e-9 && cand.wheelSpeed() < best.wheelSpeed())
                return cand;
            return best;
        } else {
            if (cand.wheelSpeed() < best.wheelSpeed() - 1e-9) return cand;
            if (Math.abs(cand.wheelSpeed() - best.wheelSpeed()) < 1e-9 && cand.hoodAngle() > best.hoodAngle())
                return cand;
            return best;
        }
    }

    private static Translation3d exitOffsetTurretFrame(double hoodRad) {
        hoodRad = MathUtil.clamp(
                hoodRad,
                Math.toDegrees(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES),
                Math.toDegrees(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES));
        ExitOffset a = ShooterConstants.EXIT_OFFSETS[0];
        ExitOffset b = ShooterConstants.EXIT_OFFSETS[ShooterConstants.EXIT_OFFSETS.length - 1];
        for (int i = 0; i < ShooterConstants.EXIT_OFFSETS.length - 1; i++) {
            if (hoodRad >= ShooterConstants.EXIT_OFFSETS[i].hoodRad()
                    && hoodRad <= ShooterConstants.EXIT_OFFSETS[i + 1].hoodRad()) {
                a = ShooterConstants.EXIT_OFFSETS[i];
                b = ShooterConstants.EXIT_OFFSETS[i + 1];
                break;
            }
        }
        double span = (b.hoodRad() - a.hoodRad());
        double t = (span <= 1e-9) ? 0.0 : (hoodRad - a.hoodRad()) / span;
        double x = a.x() + t * (b.x() - a.x());
        double y = a.y() + t * (b.y() - a.y());
        double z = a.z() + t * (b.z() - a.z());
        return new Translation3d(x, y, z);
    }

    private static double exitSpeedToWheelOmega(double exitSpeedRelMps) {
        return exitSpeedRelMps
                / (ShooterConstants.EXIT_SPEED_EFFICIENCY * ShooterConstants.SHOOTER_WHEEL_RADIUS_METERS);
    }
}
