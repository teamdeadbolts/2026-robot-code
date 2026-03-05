/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class SavedLoggedNetworkString extends LoggedNetworkString implements Tuneable<String> {
    private String key; // This is annoying
    private String lastValue = "";
    private ConfigManager configManager = ConfigManager.getInstance();

    private static final HashMap<String, SavedLoggedNetworkString> INSTANCES = new HashMap<>();

    private String immediateValue;
    private boolean hasImmediateValue = false;
    private List<Refreshable> refreshables = new ArrayList<>();
    private boolean initialized = false;

    /**
     * Get an instance of a SavedLoggedNetworkString
     *
     * @param key The key of the value
     * @param defautValue The default value
     * @return An instance of SavedLoggedNetworkString
     */
    public static synchronized SavedLoggedNetworkString get(String key, String defautValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedLoggedNetworkString(k, defautValue));
    }

    private SavedLoggedNetworkString(String key, String value) {
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
            System.out.printf("Creating new config alue %s\n", key);
            configManager.set(key, get());
            lastValue = get();
        } else {
            Object value = configManager.get(key);
            if (value instanceof String s) {
                System.out.printf("Updating %s to %s\n", key, s);
                super.set(s);
                immediateValue = s;
                hasImmediateValue = true;
                lastValue = s;
            } else {
                System.out.printf("Warning: %s is of the wrong type\n", key);
            }
        }
        //        refreshables.forEach(Refreshable::refresh);
    }

    @Override
    public void set(String value) {
        super.set(value);
        configManager.set(this.key, value);
        immediateValue = value;
        hasImmediateValue = true;
    }

    @Override
    public String get() {
        if (!initialized) {
            initFromConfig();
        }
        if (hasImmediateValue) return immediateValue;
        return super.get();
    }

    @Override
    public void periodic() {
        super.periodic();
        String c = get();
        if (!c.equals(this.lastValue)) {
            System.out.printf("Updating %s from the network to: %s\n", key, c);
            this.lastValue = c;
            immediateValue = c;
            hasImmediateValue = false;
            configManager.set(key, c);
            refreshables.forEach(Refreshable::refresh);
        }
    }
}
