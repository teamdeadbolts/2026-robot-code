/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils.tuning;

/**
 * Interface for components that can be tuned via the {@link ConfigManager}.
 * Implementing classes define how they initialize their internal state
 * using persisted configuration values.
 *
 * @param <T> The type of the value being tuned (e.g., Double, Boolean, String).
 */
public interface Tuneable<T> {
    /**
     * Initializes the component's internal values from the {@link ConfigManager}.
     * This is typically called during system startup or when a configuration is loaded.
     */
    void initFromConfig();

    void periodic();

    T get();

    void set(T value);
}
