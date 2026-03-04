/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Supplier;
import org.teamdeadbolts.subsystems.HopperSubsystem;

public class HopperCommand extends Command {
    private final HopperSubsystem hopperSubsystem;

    private final Supplier<Pose2d> poseSupplier;

    private final Supplier<Optional<HopperSubsystem.State>> requestedStateSupplier;

    private boolean lastForcedDown = false;

    public HopperCommand(
            HopperSubsystem hopperSubsystem,
            Supplier<Pose2d> poseSupplier,
            Supplier<Optional<HopperSubsystem.State>> requestedStateSupplier) {

        this.hopperSubsystem = hopperSubsystem;
        this.poseSupplier = poseSupplier;
        this.requestedStateSupplier = requestedStateSupplier;

        addRequirements(hopperSubsystem);
    }

    @Override
    public void initialize() {
        hopperSubsystem.setState(HopperSubsystem.State.HOLD);
    }

    @Override
    public void execute() {
        // Don’t do field-based logic when we don't have a real field context (optional)
        boolean enabled = DriverStation.isEnabled();
        Pose2d pose = poseSupplier.get();

        boolean forceDown = enabled && isInAutoDownZone(pose.getTranslation());

        if (forceDown) {
            // Pick FAST_DOWN vs SLOW_DOWN as you want; FAST_DOWN shown.
            setIfChanged(HopperSubsystem.State.FAST_DOWN);
            lastForcedDown = true;
            return;
        }

        lastForcedDown = false;

        // Otherwise, honor requested state if present, else HOLD.
        Optional<HopperSubsystem.State> requested = requestedStateSupplier.get();
        HopperSubsystem.State desired = requested.orElse(HopperSubsystem.State.HOLD);

        // Safety: if someone accidentally requests HOLD? that's fine. If they request FAST_DOWN/UP,
        // your subsystem already protects itself with limit switches and returns to HOLD.
        setIfChanged(desired);
    }

    private void setIfChanged(HopperSubsystem.State desired) {
        if (hopperSubsystem.getState() != desired) {
            hopperSubsystem.setState(desired);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // default commands should never finish
    }

    /**
     * Define your "auto-down" regions here.
     *
     * This example uses rectangles (axis-aligned) in FIELD METERS.
     * You can add as many zones as you want.
     */
    private boolean isInAutoDownZone(Translation2d robotPosMeters) {
        double x = robotPosMeters.getX();
        double y = robotPosMeters.getY();

        // Example zones (EDIT THESE):
        // Zone A: rectangle from (xMin,yMin) to (xMax,yMax)
        if (inRect(x, y, /*xMin*/ 1.0, /*xMax*/ 2.0, /*yMin*/ 4.5, /*yMax*/ 6.0)) {
            return true;
        }

        // Zone B:
        if (inRect(x, y, /*xMin*/ 12.0, /*xMax*/ 13.5, /*yMin*/ 1.0, /*yMax*/ 2.5)) {
            return true;
        }

        return false;
    }

    private boolean inRect(double x, double y, double xMin, double xMax, double yMin, double yMax) {
        return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
    }
}
