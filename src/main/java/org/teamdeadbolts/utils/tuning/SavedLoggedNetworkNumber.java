/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SavedLoggedNetworkNumber extends LoggedNetworkNumber implements Tuneable<Double> {
    private String key; // This is annoying
    private double lastValue = 0.0;
    private ConfigManager configManager = ConfigManager.getInstance();

    // Some more hackery
    private double immediateValue;
    private boolean hasImmediateValue = false;
    private List<Refreshable> refreshables = new ArrayList<>();
    private boolean initialized = false;

    private static final HashMap<String, SavedLoggedNetworkNumber> INSTANCES = new HashMap<>();

    /**
     * Get an instance of a SavedLoggedNetworkNumber
     *
     * @param key The key of the value
     * @param defaultValue The default value
     * @return An instance of SavedLoggedNetworkNumber
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

        //        this.refreshables.addAll(Arrays.asList(refreshables));
    }

    public void addRefreshable(Refreshable refreshable) {
        refreshables.add(refreshable);
    }

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
