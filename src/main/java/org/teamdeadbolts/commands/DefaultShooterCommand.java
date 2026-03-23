/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.utils.StatefulSubsystem.Priority;

/**
 * The default command for the shooter, active when no other shooter commands are running.
 * Manages targeting behavior based on the robot's current field position.
 */
public class DefaultShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final RobotState robotState = RobotState.getInstance();

    /**
     * @param shooterSubsystem The instance of {@link ShooterSubsystem}
     */
    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotState.getRobotPose().toPose2d();

        // If in a low zone (e.g., trench or tower), disable the shooter to avoid collisions.
        // Otherwise, track april tags
        if (ZoneConstants.isInLowZone(robotPose.getTranslation())) {
            shooterSubsystem.setState(ShooterSubsystem.State.OFF, Priority.NORMAL);
        } else {
            shooterSubsystem.setState(ShooterSubsystem.State.APRILTAG_TRACK, Priority.NORMAL);
        }
    }
}
