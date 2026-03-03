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

    /**
     * Get an instance of a SavedLoggedNetworkString
     *
     * @param key The key of the value
     * @param defautValue The default value
     * @return An instance of SavedLoggedNetworkString
     */
    public static synchronized SavedLoggedNetworkString get(
            String key, String defautValue, Refreshable... refreshables) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedLoggedNetworkString(k, defautValue, refreshables));
    }

    private SavedLoggedNetworkString(String key, String value, Refreshable... refreshables) {
        super(key, value);
        this.key = key;
        this.immediateValue = value;
        this.hasImmediateValue = true;

        configManager.registerTunable(this);

        for (Refreshable r : refreshables) {
            this.refreshables.add(r);
        }
    }

    @Override
    public void initFromConfig() {
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
    }

    @Override
    public void set(String value) {
        super.set(value);
        configManager.set(this.key, value);
        immediateValue = value;
        hasImmediateValue = true;
        refreshables.forEach(Refreshable::refresh);
    }

    @Override
    public String get() {
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
        }
    }
}
