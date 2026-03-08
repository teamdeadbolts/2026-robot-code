/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;

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
        if (isInLowZone(robotPose.getTranslation())) {
            shooterSubsystem.setState(ShooterSubsystem.State.OFF);
        } else {
            shooterSubsystem.setState(ShooterSubsystem.State.APRILTAG_TRACK);
        }
    }

    /**
     * Checks if the robot is currently within a "low zone" where the shooter must be disabled.
     * * @param robotPosMeters The current field-relative position of the robot in meters.
     * @return True if the robot is in a restricted zone.
     */
    private boolean isInLowZone(Translation2d robotPosMeters) {
        return ZoneConstants.BLUE_BOTTOM_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_BOTTOM_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.BLUE_TOP_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_TOP_TRENCH_ZONE.contains(robotPosMeters);
    }
}
