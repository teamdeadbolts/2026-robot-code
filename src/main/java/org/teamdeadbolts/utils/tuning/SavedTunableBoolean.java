/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A decoupled tunable boolean that persists to ConfigManager.
 * Uses raw key paths and is designed for multi-threaded use.
 */
public class SavedTunableBoolean implements Tuneable<Boolean> {
    private final String key;
    private final NetworkTableEntry entry;
    private final ConfigManager configManager = ConfigManager.getInstance();
    private final List<Refreshable> refreshables = new ArrayList<>();

    private volatile boolean lastValue;
    private volatile boolean initialized = false;
    private final boolean defaultValue;

    private static final Map<String, SavedTunableBoolean> INSTANCES = new HashMap<>();

    public static synchronized SavedTunableBoolean get(String key, boolean defaultValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedTunableBoolean(k, defaultValue));
    }

    private SavedTunableBoolean(String key, boolean defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.entry = NetworkTableInstance.getDefault().getEntry(key);

        if (!entry.exists()) {
            entry.setBoolean(defaultValue);
        }

        this.lastValue = entry.getBoolean(defaultValue);
        configManager.registerTunable(this);
    }

    public void addRefreshable(Refreshable refreshable) {
        refreshables.add(refreshable);
    }

    @Override
    public void initFromConfig() {
        if (initialized) return;

        boolean valueToSet;
        if (!configManager.contains(key)) {
            valueToSet = entry.getBoolean(defaultValue);
            configManager.set(key, valueToSet);
        } else {
            Object configValue = configManager.get(key);
            valueToSet = (configValue instanceof Boolean b) ? b : entry.getBoolean(defaultValue);
        }

        entry.setBoolean(valueToSet);
        this.lastValue = valueToSet;
        this.initialized = true;
        notifyRefreshables();
    }

    @Override
    public void set(Boolean value) {
        entry.setBoolean(value);
        configManager.set(this.key, value);
        this.lastValue = value;
        notifyRefreshables();
    }

    @Override
    public Boolean get() {
        if (!initialized) {
            initFromConfig();
        }
        return lastValue;
    }

    @Override
    public void periodic() {
        if (!initialized) return;

        boolean currentValue = entry.getBoolean(defaultValue);
        if (currentValue != lastValue) {
            System.out.println("Updating " + key + " from " + lastValue + " to " + currentValue);

            this.lastValue = currentValue;
            this.configManager.set(key, currentValue);
            notifyRefreshables();
        }
    }

    private void notifyRefreshables() {
        refreshables.forEach(Refreshable::refresh);
    }
}
