/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An abstract base class for subsystems that operate on a finite set of discrete states.
 * Provides a standardized way to track the current state and perform actions when
 * transitioning between them.
 *
 * @param <S> The enum representing the valid states for the subsystem.
 */
public abstract class StatefulSubsystem<S extends Enum<S>> extends SubsystemBase {

    /**
     * Defines the priority levels for state transitions.
     * Higher priorites override lower ones every cycle
     */
    public enum Priority {
        LOW,
        NORMAL,
        HIGH,
        CRITICAL;
    }

    protected S targetState;

    protected S requestedState;
    private Priority currentPriority = null;

    /**
     * Attemps to set the subsystem state with a given priority.
     * If the priority is higher than the current priority, the state change is queued.
     * If the state changes, the {@link #onStateChange} hook is triggered.
     *
     * @param newState The desired state for the subsystem.
     * @param priority The priority level for the state change.
     */
    public final void setState(S newState, Priority priority) {
        if (currentPriority == null || priority.ordinal() >= currentPriority.ordinal()) {
            this.requestedState = newState;
            this.currentPriority = priority;
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

    /**
     * Normal periodic logic for the subsystem.
     */
    protected abstract void subsystemPeriodic();

    @Override
    public final void periodic() {
        subsystemPeriodic();
        if (requestedState != null && requestedState != targetState) {
            onStateChange(requestedState, targetState);
            targetState = requestedState;
        }

        currentPriority = null;
        requestedState = null;
    }
}
