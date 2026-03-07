/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.HopperSubsystem;

/**
 * The default command for the hopper, runs whenever nothing else is trying to use the hopper subsystem
 */
public class DefaultHopperCommand extends Command {
    private final HopperSubsystem hopperSubsystem;

    /**
     * @param hopperSubsystem The instance of {@link HopperSubsystem}
     */
    public DefaultHopperCommand(HopperSubsystem hopperSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        this.hopperSubsystem.setState(HopperSubsystem.State.HOLD);
    }

    @Override
    public void execute() {
        Pose2d robotPose = RobotState.getInstance()
                .getRobotPose()
                .toPose2d(); // Get the current robot pose from the state of the whole robot

        if (isInLowZone(robotPose.getTranslation()))
            hopperSubsystem.setState(HopperSubsystem.State.DOWN); // If inside a down zone it should drop the lid
        else
            hopperSubsystem.setState(
                    HopperSubsystem.State.HOLD); // Otherwise it should hold the lid at its current position
    }

    /**
     * Check if the robot is in a zone where the lid needs to be down
     * Useally to avoid collision.
     * @param robotPosMeters The position of the field relative robot in meters
     * @return True if the robot is in a zone where the lid needs to be down
     */
    private boolean isInLowZone(Translation2d robotPosMeters) {
        return ZoneConstants.BLUE_BOTTOM_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_BOTTOM_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.BLUE_TOP_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_TOP_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.BLUE_TOWER_CLEARANCE_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_TOWER_CLEARANCE_ZONE.contains(robotPosMeters);
    }
}
