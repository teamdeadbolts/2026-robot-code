/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * A wrapper for {@link LoggedNetworkBoolean} that persists boolean values to the {@link ConfigManager}.
 * Allows for persistent state toggling (e.g., enabling/disabling features) across robot reboots.
 */
public class SavedLoggedNetworkBoolean extends LoggedNetworkBoolean implements Tuneable<Boolean> {
    private final String key;
    private boolean lastValue = false;
    private final ConfigManager configManager = ConfigManager.getInstance();

    private boolean immediateValue;
    private boolean hasImmediateValue = false;
    private final List<Refreshable> refreshables = new ArrayList<>();
    private boolean initialized = false;

    private static final HashMap<String, SavedLoggedNetworkBoolean> INSTANCES = new HashMap<>();

    /**
     * Gets or creates a singleton instance for a specific network table key.
     * @param key The network table key.
     * @param defaultValue The default value if no config exists.
     * @return The existing or new instance.
     */
    public static synchronized SavedLoggedNetworkBoolean get(String key, boolean defaultValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedLoggedNetworkBoolean(k, defaultValue));
    }

    private SavedLoggedNetworkBoolean(String key, boolean value) {
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
            if (value instanceof Boolean b) {
                System.out.printf("Updating %s to %s\n", key, b);
                super.set(b);
                lastValue = b;
                immediateValue = b;
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
    public void set(boolean value) {
        super.set(value);
        this.immediateValue = value;
        this.hasImmediateValue = true;
        configManager.set(this.key, value);
    }

    @Override
    public boolean get() {
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
        boolean c = super.get();
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
