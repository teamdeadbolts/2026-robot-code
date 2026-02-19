/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class DriveCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier rotationSupplier;
    private boolean fieldRelative;

    private final SavedLoggedNetworkNumber controllerDeadband =
            SavedLoggedNetworkNumber.get("Tuning/Drive/ControllerDeadband", 0.08);

    private final SavedLoggedNetworkNumber maxRobotSpeed = SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotSpeed", 1.0);
    private final SavedLoggedNetworkNumber bumbSpeed = SavedLoggedNetworkNumber.get("Tuning/Drive/BumpSpeed", 0.5);

    private final SavedLoggedNetworkNumber maxRobotAnglarSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Drive/MaxRobotAngluarSpeed", 1.0);

    /**
     * Command to drive swerve
     *
     * @param swerveSubsystem The instance of {@link SwerveSubsystem}
     * @param forwardSupplier A supplier for forward motion (in <strong>%</strong>)
     * @param sidewaysSupplier A supplier for sideways motion (in <strong>%/strong>)
     * @param rotationSuplier A supplier for rotaional motion (in <strong>%</strong>)
     * @param fieldRelative Weather or not to drive the robot field relative
     */
    public DriveCommand(
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier forwardSupplier,
            DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSuplier,
            boolean fieldRelative) {
        this.swerveSubsystem = swerveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.sidewaysSupplier = sidewaysSupplier;
        this.rotationSupplier = rotationSuplier;
        this.fieldRelative = fieldRelative;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Translation2d robotTrans =
                RobotState.getInstance().getRobotPose().toPose2d().getTranslation();

        double forwardPercent = MathUtil.applyDeadband(-forwardSupplier.getAsDouble(), controllerDeadband.get(), 1);
        double sidewaysPercent = MathUtil.applyDeadband(-sidewaysSupplier.getAsDouble(), controllerDeadband.get(), 1);
        double rotationPercent = MathUtil.applyDeadband(-rotationSupplier.getAsDouble(), controllerDeadband.get(), 1);

        double forwardMps = forwardPercent * maxRobotSpeed.get();
        double sidewaysMps = sidewaysPercent * maxRobotSpeed.get();
        double rotationMps = rotationPercent * Units.degreesToRadians(maxRobotAnglarSpeed.get());

        if (SwerveConstants.RED_TOP_BUMP_ZONE.contains(robotTrans)
                || SwerveConstants.RED_BOTTOM_BUMP_ZONE.contains(robotTrans)
                || SwerveConstants.BLUE_TOP_BUMP_ZONE.contains(robotTrans)
                || SwerveConstants.BLUE_BOTTOM_BUMP_ZONE.contains(robotTrans)) {
            forwardMps = forwardMps * bumbSpeed.get();
            sidewaysMps = sidewaysMps * bumbSpeed.get();
        }

        Logger.recordOutput("Drive/ForwardPercent", forwardSupplier.getAsDouble());
        Logger.recordOutput("Drive/SidewaysPercent", sidewaysSupplier.getAsDouble());
        Logger.recordOutput("Drive/AnglePercent", rotationSupplier.getAsDouble());

        swerveSubsystem.drive(new ChassisSpeeds(forwardMps, sidewaysMps, rotationMps), fieldRelative, false, false);
    }
}
