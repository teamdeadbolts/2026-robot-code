/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.HopperSubsystem;

public class DefaultHopperCommand extends Command {
    private final HopperSubsystem hopperSubsystem;

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
        Pose2d robotPose = RobotState.getInstance().getRobotPose().toPose2d();

        if (isInAutoDownZone(robotPose.getTranslation())) hopperSubsystem.setState(HopperSubsystem.State.DOWN);
        else hopperSubsystem.setState(HopperSubsystem.State.HOLD);
    }

    private boolean isInAutoDownZone(Translation2d robotPosMeters) {
        return ZoneConstants.BLUE_BOTTOM_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_BOTTOM_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.BLUE_TOP_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_TOP_TRENCH_ZONE.contains(robotPosMeters)
                || ZoneConstants.BLUE_TOWER_ZONE.contains(robotPosMeters)
                || ZoneConstants.RED_TOWER_ZONE.contains(robotPosMeters);
    }
}
