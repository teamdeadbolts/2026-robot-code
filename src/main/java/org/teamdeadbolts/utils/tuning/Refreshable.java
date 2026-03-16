/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils.tuning;

/**
 * Interface for components that need to respond to configuration or network updates.
 * Used for reactive updates to system parameters like PID constants or motor configurations.
 */
public interface Refreshable {
    /**
     * Called when a bound {@link Tuneable} value is updated, allowing the implementing
     * component to refresh its internal state or hardware settings.
     */
    void refresh();
}
