/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import java.util.TreeMap;

public class ExtrapolatingDoubleMap {
    private final TreeMap<Double, Double> map = new TreeMap<>();
    private final double maxHistroyMs;

    public ExtrapolatingDoubleMap(double maxHistoryMs) {
        this.maxHistroyMs = maxHistoryMs;
    }

    public void put(double timestamp, double value) {
        map.put(timestamp, value);
        cleanUp(timestamp - maxHistroyMs);
    }

    public double get(double timestamp) {
        if (map.isEmpty()) return 0;
        if (map.size() == 1) return map.firstEntry().getValue();

        if (map.containsKey(timestamp)) {
            return map.get(timestamp);
        }

        Double lower = map.lowerKey(timestamp);
        Double higher = map.higherKey(timestamp);

        if (higher == null) {
            Double last = map.lastKey();
            Double prev = map.lowerKey(last);
            return linearMath(prev, map.get(prev), last, map.get(last), timestamp);
        }

        if (lower == null) {
            Double first = map.firstKey();
            Double next = map.higherKey(first);
            return linearMath(first, map.get(first), next, map.get(next), timestamp);
        }

        return linearMath(lower, map.get(lower), higher, map.get(higher), timestamp);
    }

    public void clear() {
        map.clear();
    }

    public void isEmpty() {
        map.isEmpty();
    }

    public int size() {
        return map.size();
    }

    private void cleanUp(double cutoffTimeMs) {
        while (map.firstKey() < cutoffTimeMs) {
            map.remove(map.firstKey());
        }
    }

    private double linearMath(double x1, double y1, double x2, double y2, double targetX) {
        if (Math.abs(x2 - x1) <= 1e-6) return y2;
        double slope = (y2 - y1) / (x2 - x1);
        return y1 + slope * (targetX - x1);
    }
}
