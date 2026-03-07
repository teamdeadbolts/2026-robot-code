/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A wrapper for {@link LoggedNetworkNumber} that persists values to the {@link ConfigManager}.
 * This allows network-based tuning (e.g., AdvantageScope) to persist across robot reboots.
 */
public class SavedLoggedNetworkNumber extends LoggedNetworkNumber implements Tuneable<Double> {
    private final String key;
    private double lastValue = 0.0;
    private final ConfigManager configManager = ConfigManager.getInstance();

    private double immediateValue;
    private boolean hasImmediateValue = false;
    private final List<Refreshable> refreshables = new ArrayList<>();
    private boolean initialized = false;

    private static final HashMap<String, SavedLoggedNetworkNumber> INSTANCES = new HashMap<>();

    /**
     * Gets or creates a singleton instance for a specific network table key.
     * @param key The network table key.
     * @param defaultValue The default value if no config exists.
     * @return The existing or new instance.
     */
    public static synchronized SavedLoggedNetworkNumber get(String key, double defaultValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedLoggedNetworkNumber(k, defaultValue));
    }

    private SavedLoggedNetworkNumber(String key, double value) {
        super(key, value);
        this.key = key;
        this.immediateValue = value;
        this.hasImmediateValue = true;

        configManager.registerTunable(this);
    }

    /**
     * Adds a {@link Refreshable} component to be notified when this value changes.
     * @param refreshable The component to refresh.
     */
    public void addRefreshable(Refreshable refreshable) {
        refreshables.add(refreshable);
    }

    /**
     * Initializes the value from {@link ConfigManager} or sets a default if missing.
     */
    @Override
    public void initFromConfig() {
        if (initialized) return;
        initialized = true;
        if (!configManager.contains(key)) {
            System.out.printf("Creating new config value %s\n", key);
            configManager.set(key, get());
            lastValue = get();
        } else {
            Object value = configManager.get(key);
            if (value instanceof Double d) {
                System.out.printf("Updating %s to %s\n", key, d);
                super.set(d);
                lastValue = d;
                immediateValue = d;
                hasImmediateValue = true;
            } else {
                System.out.printf("Warning: %s is of the wrong type\n", key);
            }
        }
        refreshables.forEach(Refreshable::refresh);
    }

    /**
     * Updates the value locally, in the network table, and in persistent storage.
     * @param value The new value.
     */
    @Override
    public void set(double value) {
        super.set(value);
        configManager.set(this.key, value);
        this.immediateValue = value;
        this.hasImmediateValue = true;
    }

    @Override
    public double get() {
        if (!initialized) {
            initFromConfig();
        }
        if (hasImmediateValue) {
            return immediateValue;
        }
        return super.get();
    }

    /**
     * Periodically checks for network updates (e.g., from AdvantageScope) and
     * synchronizes with the config manager and registered refreshables.
     */
    @Override
    public void periodic() {
        super.periodic();
        double c = super.get();
        if (c != this.lastValue) {
            System.out.printf("Updating %s from the network to: %s\n", key, c);
            this.lastValue = c;
            immediateValue = c;
            hasImmediateValue = false;
            configManager.set(key, c);
            refreshables.forEach(Refreshable::refresh);
        }
    }
}
