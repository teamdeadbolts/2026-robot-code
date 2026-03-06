/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.HopperSubsystem;

public class HopperCommand extends Command {
    private final HopperSubsystem hopperSubsystem;

    // What the driver last requested (latched)
    private DriverRequest latchedRequest = DriverRequest.NONE;

    // Choose fast vs slow
    private final boolean goFast;

    // Optional: If true, pressing UP while in a down-zone will clear the request instead of "queueing" it.

    public HopperCommand(HopperSubsystem hopperSubsystem, boolean goFast) {
        this.hopperSubsystem = hopperSubsystem;
        this.goFast = goFast;
        addRequirements(hopperSubsystem);
    }

    private enum DriverRequest {
        NONE,
        GO_UP,
        GO_DOWN
    }

    public void requestUp() {
        latchedRequest = DriverRequest.GO_UP;
    }

    public void requestDown() {
        latchedRequest = DriverRequest.GO_DOWN;
    }

    public void clearRequest() {
        latchedRequest = DriverRequest.NONE;
    }

    @Override
    public void initialize() {
        hopperSubsystem.setState(HopperSubsystem.State.HOLD);
        latchedRequest = DriverRequest.NONE;
    }

    @Override
    public void execute() {
        boolean enabled = DriverStation.isEnabled();
        Pose2d robotPose = RobotState.getInstance().getRobotPose().toPose2d();

        boolean inZone = enabled && isInAutoDownZone(robotPose.getTranslation());

        // Priority 1: Zone override always wins
        if (inZone) {
            latchedRequest = DriverRequest.NONE;

            if (hopperSubsystem.isLowerLimitReached()) {
                setIfChanged(HopperSubsystem.State.HOLD);
            } else {
                setIfChanged(HopperSubsystem.State.FAST_DOWN);
            }
            return;
        }

        // Priority 2: run latched driver request until limit reached
        switch (latchedRequest) {
            case GO_UP:
                if (hopperSubsystem.isUpperLimitReached()) {
                    latchedRequest = DriverRequest.NONE;
                    setIfChanged(HopperSubsystem.State.HOLD);
                } else {
                    setIfChanged(goFast ? HopperSubsystem.State.FAST_UP : HopperSubsystem.State.SLOW_UP);
                }
                return;

            case GO_DOWN:
                if (hopperSubsystem.isLowerLimitReached()) {
                    latchedRequest = DriverRequest.NONE;
                    setIfChanged(HopperSubsystem.State.HOLD);
                } else {
                    setIfChanged(goFast ? HopperSubsystem.State.FAST_DOWN : HopperSubsystem.State.SLOW_DOWN);
                }
                return;

            case NONE:
            default:
                // Priority 3: default hold
                setIfChanged(HopperSubsystem.State.HOLD);
        }
    }

    private void setIfChanged(HopperSubsystem.State desired) {
        if (hopperSubsystem.getState() != desired) {
            hopperSubsystem.setState(desired);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // default command
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
