/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A decoupled tunable number that persists to ConfigManager.
 * Uses the raw key path for NetworkTables and is designed for multi-threaded use.
 */
public class SavedTunableNumber implements Tuneable<Double> {
    private final String key;
    private final NetworkTableEntry entry;
    private final ConfigManager configManager = ConfigManager.getInstance();
    private final List<Refreshable> refreshables = new ArrayList<>();

    private volatile double lastValue;
    private volatile boolean initialized = false;

    private volatile double defaultValue;
    private static final Map<String, SavedTunableNumber> INSTANCES = new HashMap<>();

    public static synchronized SavedTunableNumber get(String key, double defaultValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedTunableNumber(k, defaultValue));
    }

    private SavedTunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;

        this.entry = NetworkTableInstance.getDefault().getEntry(key);

        if (!entry.exists()) {
            entry.setDouble(defaultValue);
        }

        this.lastValue = entry.getDouble(defaultValue);
        configManager.registerTunable(this);
    }

    public void addRefreshable(Refreshable refreshable) {
        refreshables.add(refreshable);
    }

    @Override
    public void initFromConfig() {
        if (initialized) return;

        double valueToSet;
        if (!configManager.contains(key)) {
            // If not in config, use whatever is currently in the NetworkTable
            valueToSet = entry.getDouble(defaultValue);
            configManager.set(key, valueToSet);
        } else {
            // If in config, override the NetworkTable with the persisted value
            Object configValue = configManager.get(key);
            valueToSet = (configValue instanceof Double d) ? d : entry.getDouble(defaultValue);
        }

        entry.setDouble(valueToSet);
        this.lastValue = valueToSet;
        this.initialized = true;

        notifyRefreshables();
    }

    @Override
    public void set(Double value) {
        entry.setDouble(value);
        configManager.set(this.key, value);
        this.lastValue = value;
        notifyRefreshables();
    }

    @Override
    public Double get() {
        if (!initialized) {
            initFromConfig();
        }
        return lastValue;
    }

    /**
     * Checks for updates from the network. Run this on your background thread.
     */
    @Override
    public void periodic() {
        if (!initialized) return;

        double currentValue = entry.getDouble(defaultValue);
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
