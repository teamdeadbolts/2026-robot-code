/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * A wrapper for {@link LoggedNetworkString} that persists string values to the {@link ConfigManager}.
 * Useful for persistent string settings, such as mode selectors or configuration identifiers.
 */
public class SavedLoggedNetworkString extends LoggedNetworkString implements Tuneable<String> {
    private final String key;
    private String lastValue = "";
    private final ConfigManager configManager = ConfigManager.getInstance();

    private String immediateValue;
    private boolean hasImmediateValue = false;
    private final List<Refreshable> refreshables = new ArrayList<>();
    private boolean initialized = false;

    private static final HashMap<String, SavedLoggedNetworkString> INSTANCES = new HashMap<>();

    /**
     * Gets or creates a singleton instance for a specific network table key.
     * @param key The network table key.
     * @param defaultValue The default value if no config exists.
     * @return The existing or new instance.
     */
    public static synchronized SavedLoggedNetworkString get(String key, String defaultValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedLoggedNetworkString(k, defaultValue));
    }

    private SavedLoggedNetworkString(String key, String value) {
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
            if (value instanceof String s) {
                System.out.printf("Updating %s to %s\n", key, s);
                super.set(s);
                lastValue = s;
                immediateValue = s;
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
    public void set(String value) {
        super.set(value);
        configManager.set(this.key, value);
        this.immediateValue = value;
        this.hasImmediateValue = true;
    }

    @Override
    public String get() {
        if (!initialized) {
            initFromConfig();
        }
        if (hasImmediateValue) {
            return immediateValue;
        }
        return super.get();
    }

    /**
     * Periodically checks for network updates and synchronizes with the
     * config manager and registered refreshables.
     */
    @Override
    public void periodic() {
        super.periodic();
        String c = super.get();
        if (c != null && !c.equals(this.lastValue)) {
            System.out.printf("Updating %s from the network to: %s\n", key, c);
            this.lastValue = c;
            immediateValue = c;
            hasImmediateValue = false;
            configManager.set(key, c);
            refreshables.forEach(Refreshable::refresh);
        }
    }
}
