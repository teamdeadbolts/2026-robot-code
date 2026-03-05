/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;

public class DefaultShooterCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private RobotState robotState = RobotState.getInstance();

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotState.getRobotPose().toPose2d();
        // If we are in any of the trench zones put down the hood
        if (SwerveConstants.RED_BOTTOM_TRENCH_ZONE.contains(robotPose.getTranslation())
                || SwerveConstants.BLUE_BOTTOM_TRENCH_ZONE.contains(robotPose.getTranslation())
                || SwerveConstants.RED_TOP_BUMP_ZONE.contains(robotPose.getTranslation())
                || SwerveConstants.BLUE_TOP_BUMP_ZONE.contains(robotPose.getTranslation())) {
            shooterSubsystem.setState(ShooterSubsystem.State.OFF);
            return;
        }

        shooterSubsystem.setState(ShooterSubsystem.State.APRILTAG_TRACK);
    }
}
