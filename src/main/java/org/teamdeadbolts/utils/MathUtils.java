/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils;

public class MathUtils {
    /**
     * Convert meters per second to rotations per second
     *
     * @param wMPS Wheel velocity (in <strong>m/s</strong>)
     * @param c The circumference of the wheels
     * @return Wheel velocity (in <strong>rotations/s</strong>)
     */
    public static double MPSToRPS(double wMPS, double c) {
        return wMPS / c;
    }

    /**
     * Convert rotations per second to meters per second
     *
     * @param wRPS The wheel velocity (in <strong>rotations/s</strong>)
     * @param c The circumference of the wheels
     * @return Wheel velocity (in <strong>m/s</strong>)
     */
    public static double RPSToMPS(double wRPS, double c) {
        return wRPS * c;
    }

    public static double MPSToRPM(double mps, double r) {
        return (60.0 / (2.0 * Math.PI * r)) * mps;
    }
}
