/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class SavedLoggedNetworkBoolean extends LoggedNetworkBoolean implements Tuneable<Boolean> {
    private String key; // This is annoying
    private boolean lastValue = false;
    private ConfigManager configManager = ConfigManager.getInstance();

    private boolean immediateValue;
    private boolean hasImmediateValue = false;
    private List<Refreshable> refreshables = new ArrayList<>();
    private boolean initialized = false;

    private static final HashMap<String, SavedLoggedNetworkBoolean> INSTANCES = new HashMap<>();

    /**
     * Get an instance of a SavedLoggedNetworkBoolean
     *
     * @param key The key of the value
     * @param defautValue The default value
     * @return An instance of SavedLoggedNetworkBoolean
     */
    public static synchronized SavedLoggedNetworkBoolean get(String key, boolean defautValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedLoggedNetworkBoolean(k, defautValue));
    }

    private SavedLoggedNetworkBoolean(String key, boolean value) {
        super(key, value);
        this.key = key;
        this.immediateValue = value;
        this.hasImmediateValue = true;
        configManager.registerTunable(this);
    }

    public void add(Refreshable refreshable) {
        refreshables.add(refreshable);
    }

    @Override
    public void initFromConfig() {
        if (initialized) return;
        initialized = true;
        if (!configManager.contains(key)) {
            System.out.printf("Creating new config value %s\n", key);
            configManager.set(key, get());
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
        //        refreshables.forEach(Refreshable::refresh);
    }

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
