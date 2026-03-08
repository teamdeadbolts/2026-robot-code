/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Command to drive the swerve
 */
public class DriveCommand extends Command implements Refreshable {
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier rotationSupplier;
    private boolean fieldRelative;

    // For shooter fallback aiming
    private final PIDController angleController = new PIDController(0, 0, 0);

    private final SavedLoggedNetworkNumber controllerDeadband =
            SavedLoggedNetworkNumber.get("Tuning/Drive/ControllerDeadband", 0.08);

    private final SavedLoggedNetworkNumber maxRobotSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotSpeed", 1.0);
    private final SavedLoggedNetworkNumber bumbSpeed = SavedLoggedNetworkNumber.get("Tuning/Drive/BumpSpeed", 0.5);

    private final SavedLoggedNetworkNumber maxRobotAnglarSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotAngluarSpeed", 1.0);

    private final SavedLoggedNetworkNumber defaultDrivePercent =
            SavedLoggedNetworkNumber.get("Tuning/Drive/DefaultDrivePercent", 0.5);
    private final SavedLoggedNetworkNumber slowDrivePercent =
            SavedLoggedNetworkNumber.get("Tuning/Drive/SlowDrivePercent", 0.28);

    private final SavedLoggedNetworkNumber angleControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Drive/AngleController/kP", 0);

    private boolean fast;
    private boolean slow;
    /**
     * Command to drive swerve
     *
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param forwardSupplier A supplier for forward motion (in <strong>%</strong>)
     * @param sidewaysSupplier A supplier for sideways motion (in <strong>%/strong>)
     * @param rotationSuplier A supplier for rotaional motion (in <strong>%</strong>)
     * @param fieldRelative Weather or not to drive the robot field relative
     * @param fast Weather or not to drive at full speed (fast)
     * @param slow Weather or not to drive even slower than normal
     */
    public DriveCommand(
            SwerveSubsystem swerveSubsystem,
            ShooterSubsystem shooterSubsystem,
            DoubleSupplier forwardSupplier,
            DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSuplier,
            boolean fieldRelative,
            boolean fast,
            boolean slow) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.sidewaysSupplier = sidewaysSupplier;
        this.rotationSupplier = rotationSuplier;
        this.fieldRelative = fieldRelative;
        this.fast = fast;
        this.slow = slow;

        angleControllerP.addRefreshable(this);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void refresh() {
        angleController.setP(angleControllerP.get());
    }

    @Override
    public void execute() {
        Pose2d robotPose = RobotState.getInstance().getRobotPose().toPose2d();
        Translation2d robotTrans = robotPose.getTranslation();

        double forwardPercent = MathUtil.applyDeadband(-forwardSupplier.getAsDouble(), controllerDeadband.get(), 1);
        double sidewaysPercent = MathUtil.applyDeadband(-sidewaysSupplier.getAsDouble(), controllerDeadband.get(), 1);
        double rotationPercent = MathUtil.applyDeadband(-rotationSupplier.getAsDouble(), controllerDeadband.get(), 1);

        double forwardMps;
        double sidewaysMps;
        double rotationRps;

        if (fast) {
            slow = false;
            forwardMps = forwardPercent * maxRobotSpeed.get();
            sidewaysMps = sidewaysPercent * maxRobotSpeed.get();
            rotationRps = rotationPercent * Units.degreesToRadians(maxRobotAnglarSpeed.get());
        } else if (slow) {
            forwardMps = forwardPercent * maxRobotSpeed.get() * slowDrivePercent.get();
            sidewaysMps = sidewaysPercent * maxRobotSpeed.get() * slowDrivePercent.get();
            rotationRps = rotationPercent * Units.degreesToRadians(maxRobotAnglarSpeed.get()) * slowDrivePercent.get();
        } else {
            forwardMps = forwardPercent * maxRobotSpeed.get() * slowDrivePercent.get();
            sidewaysMps = sidewaysPercent * maxRobotSpeed.get() * slowDrivePercent.get();
            rotationRps = rotationPercent * Units.degreesToRadians(maxRobotAnglarSpeed.get()) * slowDrivePercent.get();
        }

        if (ZoneConstants.RED_TOP_BUMP_ZONE.contains(robotTrans)
                || ZoneConstants.RED_BOTTOM_BUMP_ZONE.contains(robotTrans)
                || ZoneConstants.BLUE_TOP_BUMP_ZONE.contains(robotTrans)
                || ZoneConstants.BLUE_BOTTOM_BUMP_ZONE.contains(robotTrans)) {
            forwardMps = forwardMps * bumbSpeed.get();
            sidewaysMps = sidewaysMps * bumbSpeed.get();
        }

        Optional<Rotation2d> aimAngle = shooterSubsystem.getFallbackChassisTargetAngle();
        if (aimAngle.isPresent()) {
            rotationRps = angleController.calculate(
                    robotPose.getRotation().getRadians(), aimAngle.get().getRadians());
            Logger.recordOutput("Drive/ShooterAimFallback", true);
        } else {
            Logger.recordOutput("Drive/ShooterAimFallback", false);
        }

        Logger.recordOutput("Drive/ForwardPercent", forwardSupplier.getAsDouble());
        Logger.recordOutput("Drive/SidewaysPercent", sidewaysSupplier.getAsDouble());
        Logger.recordOutput("Drive/AnglePercent", rotationSupplier.getAsDouble());
        Logger.recordOutput("Drive/ForwardMps", forwardMps);
        Logger.recordOutput("Drive/SidewaysMps", sidewaysMps);
        Logger.recordOutput("Drive/RotationRps", rotationRps);

        swerveSubsystem.drive(new ChassisSpeeds(forwardMps, sidewaysMps, rotationRps), fieldRelative, true, false);
    }
}
