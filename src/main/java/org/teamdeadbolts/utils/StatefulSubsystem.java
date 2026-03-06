/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * An abstract base class for subsystems that operate on a finite set of discrete states.
 * Provides a standardized way to track the current state and perform actions when
 * transitioning between them.
 *
 * @param <S> The enum representing the valid states for the subsystem.
 */
public abstract class StatefulSubsystem<S extends Enum<S>> extends SubsystemBase {
    @AutoLogOutput
    protected S targetState;

    /**
     * Updates the subsystem state and triggers the {@link #onStateChange} hook
     * if the state has transitioned to a new value.
     *
     * @param newState The desired state for the subsystem.
     */
    public final void setState(S newState) {
        if (this.targetState != newState) {
            onStateChange(newState, targetState);
            targetState = newState;
        }
    }

    /**
     * @return The current subsystem state.
     */
    public S getState() {
        return targetState;
    }

    /**
     * Hook method triggered when the subsystem transitions from one state to another.
     * Implementations should handle logic required for entering the new state (e.g.,
     * resting states (like a PID)).
     *
     * @param to   The new state.
     * @param from The previous state.
     */
    protected abstract void onStateChange(S to, S from);
}
