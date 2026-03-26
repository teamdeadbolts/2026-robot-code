/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils;

/**
 * Provides kinematic conversion utilities and common mathematical checks
 * for robot drive calculations.
 */
public class MathUtils {
    /**
     * Converts wheel velocity from meters per second to rotations per second.
     *
     * @param wMPS Wheel velocity in m/s.
     * @param c The circumference of the wheels in meters.
     * @return Wheel velocity in rotations/s.
     */
    public static double MPSToRPS(double wMPS, double c) {
        return wMPS / c;
    }

    /**
     * Converts wheel velocity from rotations per second to meters per second.
     *
     * @param wRPS The wheel velocity in rotations/s.
     * @param c The circumference of the wheels in meters.
     * @return Wheel velocity in m/s.
     */
    public static double RPSToMPS(double wRPS, double c) {
        return wRPS * c;
    }

    /**
     * Converts linear velocity to angular velocity (RPM).
     * * @param mps Linear velocity in m/s.
     * @param r The radius of the wheels in meters.
     * @return Angular velocity in RPM.
     */
    public static double MPSToRPM(double mps, double r) {
        // Calculation: $RPM = \frac{v}{2\pi r} \times 60$
        return (60.0 / (2.0 * Math.PI * r)) * mps;
    }

    /**
     * Checks if a value falls within a specified inclusive range.
     *
     * @param value The value to check.
     * @param min The lower bound.
     * @param max The upper bound.
     * @return True if the value is within [min, max].
     */
    public static boolean inRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    /**
     * Calculates the shortest distance between two angles in radians. (-pi to pi)
     * @param current The current encoder position
     * @param target The desired position
     * @return The shortest distance
     */
    public static double getShortestDistance(double current, double target) {
        double diff = target - current;

        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff <= -Math.PI) diff += 2 * Math.PI;

        return diff;
    }
}
