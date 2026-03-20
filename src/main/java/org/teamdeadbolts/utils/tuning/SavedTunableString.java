/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A decoupled tunable string that persists to ConfigManager.
 * Uses raw key paths and is designed for multi-threaded use.
 */
public class SavedTunableString implements Tuneable<String> {
    private final String key;
    private final NetworkTableEntry entry;
    private final ConfigManager configManager = ConfigManager.getInstance();
    private final List<Refreshable> refreshables = new ArrayList<>();

    private volatile String lastValue;
    private volatile boolean initialized = false;
    private final String defaultValue;

    private static final Map<String, SavedTunableString> INSTANCES = new HashMap<>();

    public static synchronized SavedTunableString get(String key, String defaultValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedTunableString(k, defaultValue));
    }

    private SavedTunableString(String key, String defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.entry = NetworkTableInstance.getDefault().getEntry(key);

        if (!entry.exists()) {
            entry.setString(defaultValue);
        }

        this.lastValue = entry.getString(defaultValue);
        configManager.registerTunable(this);
    }

    public void addRefreshable(Refreshable refreshable) {
        refreshables.add(refreshable);
    }

    @Override
    public void initFromConfig() {
        if (initialized) return;

        String valueToSet;
        if (!configManager.contains(key)) {
            valueToSet = entry.getString(defaultValue);
            configManager.set(key, valueToSet);
        } else {
            Object configValue = configManager.get(key);
            valueToSet = (configValue instanceof String s) ? s : entry.getString(defaultValue);
        }

        entry.setString(valueToSet);
        this.lastValue = valueToSet;
        this.initialized = true;
        notifyRefreshables();
    }

    @Override
    public void set(String value) {
        entry.setString(value);
        configManager.set(this.key, value);
        this.lastValue = value;
        notifyRefreshables();
    }

    @Override
    public String get() {
        if (!initialized) {
            initFromConfig();
        }
        return lastValue;
    }

    @Override
    public void periodic() {
        if (!initialized) return;

        String currentValue = entry.getString(defaultValue);
        if (currentValue != null && !currentValue.equals(lastValue)) {
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
