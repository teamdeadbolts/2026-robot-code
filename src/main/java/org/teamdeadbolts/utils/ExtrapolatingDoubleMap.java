/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import java.util.TreeMap;

/**
 * A time-series utility that maintains a sliding window of historical data points
 * and provides interpolated or extrapolated values for any requested timestamp.
 */
public class ExtrapolatingDoubleMap {
    private final TreeMap<Double, Double> map = new TreeMap<>();
    private final double maxHistoryMs;

    /**
     * @param maxHistoryMs The duration in milliseconds to retain data points.
     */
    public ExtrapolatingDoubleMap(double maxHistoryMs) {
        this.maxHistoryMs = maxHistoryMs;
    }

    /**
     * Adds a data point and removes entries older than the defined history window.
     * @param timestamp The timestamp of the data point.
     * @param value The value at that timestamp.
     */
    public void put(double timestamp, double value) {
        map.put(timestamp, value);
        cleanUp(timestamp - maxHistoryMs);
    }

    /**
     * Retrieves the value for a given timestamp. Interpolates between points
     * if the exact timestamp is missing, or extrapolates based on the trend
     * of the last two points if the timestamp is outside the range.
     * * @param timestamp The target timestamp.
     * @return The interpolated or extrapolated value.
     */
    public double get(double timestamp) {
        if (map.isEmpty()) return 0;
        if (map.size() == 1) return map.firstEntry().getValue();

        if (map.containsKey(timestamp)) {
            return map.get(timestamp);
        }

        Double lower = map.lowerKey(timestamp);
        Double higher = map.higherKey(timestamp);

        // Extrapolate forward if timestamp is newer than latest data
        if (higher == null) {
            Double last = map.lastKey();
            Double prev = map.lowerKey(last);
            return linearMath(prev, map.get(prev), last, map.get(last), timestamp);
        }

        // Extrapolate backward if timestamp is older than earliest data
        if (lower == null) {
            Double first = map.firstKey();
            Double next = map.higherKey(first);
            return linearMath(first, map.get(first), next, map.get(next), timestamp);
        }

        // Interpolate between surrounding points
        return linearMath(lower, map.get(lower), higher, map.get(higher), timestamp);
    }

    /** Clears all historical data. */
    public void clear() {
        map.clear();
    }

    public boolean isEmpty() {
        return map.isEmpty();
    }

    /** @return The number of stored data points. */
    public int size() {
        return map.size();
    }

    /**
     * Removes data points older than the provided threshold.
     * @param cutoffTimeMs The expiration timestamp.
     */
    private void cleanUp(double cutoffTimeMs) {
        while (!map.isEmpty() && map.firstKey() < cutoffTimeMs) {
            map.remove(map.firstKey());
        }
    }

    /**
     * Calculates the value at targetX using linear interpolation/extrapolation.
     */
    private double linearMath(double x1, double y1, double x2, double y2, double targetX) {
        if (Math.abs(x2 - x1) <= 1e-6) return y2;
        double slope = (y2 - y1) / (x2 - x1);
        return y1 + slope * (targetX - x1);
    }
}
