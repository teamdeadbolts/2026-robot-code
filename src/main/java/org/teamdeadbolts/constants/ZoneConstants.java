/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import edu.wpi.first.math.geometry.Translation2d;
import org.teamdeadbolts.utils.Zone;

public class ZoneConstants {
    // TODO: Find these
    public static final Zone BLUE_TOP_BUMP_ZONE = new Zone(
            new Translation2d(3.71, 6.61),
            new Translation2d(3.71, 5.027),
            new Translation2d(5.536, 5.027),
            new Translation2d(5.536, 6.61));
    public static final Zone BLUE_BOTTOM_BUMP_ZONE = new Zone(
            new Translation2d(3.71, 3.037),
            new Translation2d(3.71, 1.42),
            new Translation2d(5.536, 1.42),
            new Translation2d(5.536, 3.037));
    public static final Zone BLUE_BOTTOM_TRENCH_ZONE = new Zone(
            new Translation2d(3.63, 1.42),
            new Translation2d(3.63, -0.32),
            new Translation2d(5.586, -0.32),
            new Translation2d(5.586, 1.42));
    public static final Zone BLUE_TOP_TRENCH_ZONE = new Zone(
            new Translation2d(3.63, 7.72),
            new Translation2d(3.63, 6.61),
            new Translation2d(5.586, 6.61),
            new Translation2d(5.586, 7.72));

    public static final Zone RED_TOP_BUMP_ZONE = new Zone(
            new Translation2d(11.05, 6.625),
            new Translation2d(11.05, 4.977),
            new Translation2d(12.738, 4.977),
            new Translation2d(12.738, 6.625));
    public static final Zone RED_BOTTOM_BUMP_ZONE = new Zone(
            new Translation2d(11.05, 3.068),
            new Translation2d(11.05, 1.439),
            new Translation2d(12.738, 1.439),
            new Translation2d(12.738, 3.068));
    public static final Zone RED_BOTTOM_TRENCH_ZONE = new Zone(
            new Translation2d(10.95, 1.437),
            new Translation2d(10.95, 0.384),
            new Translation2d(12.8, 0.384),
            new Translation2d(12.8, 1.437));
    public static final Zone RED_TOP_TRENCH_ZONE = new Zone(
            new Translation2d(10.91, 8.4),
            new Translation2d(10.91, 6.63),
            new Translation2d(12.83, 6.63),
            new Translation2d(12.83, 8.4));

    public static final Zone RED_SCORE_ZONE = new Zone(
            new Translation2d(11.97 - 0.4, 7.939 + 0.4),
            new Translation2d(11.97 - 0.4, 0.16 - 0.4),
            new Translation2d(16.45 + 0.4, 0.16 - 0.4),
            new Translation2d(16.45 + 0.4, 7.939 + 0.4));
    public static final Zone BLUE_SCORE_ZONE = new Zone(
            new Translation2d(0.0 - 0.4, 7.95 + 0.4),
            new Translation2d(0.0 - 0.4, 0.0 - 0.4),
            new Translation2d(4.4095 + 0.4, 0.0 - 0.4),
            new Translation2d(4.4095 + 0.4, 7.95 + 0.4));

    public static final Zone RED_HUB_CLEARANCE_ZONE = new Zone(
            new Translation2d(11.342 - 0.4, 4.617 + 0.4),
            new Translation2d(11.33 - 0.4, 3.455 - 0.4),
            new Translation2d(12.514 + 0.67 + 0.4, 3.467 - 0.4),
            new Translation2d(12.514 + 0.67 + 0.4, 4.633 + 0.4));
    public static final Zone BLUE_HUB_CLEARANCE_ZONE = new Zone(
            new Translation2d(4.0121 - 0.67056 - 0.4, 4.60 + 0.4),
            new Translation2d(4.0121 - 0.67056 - 0.4, 3.472 - 0.4),
            new Translation2d(5.212 + 0.4, 3.472 - 0.4),
            new Translation2d(5.212 + 0.4, 4.60 + 0.4));

    public static final Zone RED_TOWER_ZONE = new Zone(
            new Translation2d(15.455 - 0.4, 4.962 + 0.4),
            new Translation2d(15.455 - 0.4, 3.682 - 0.4),
            new Translation2d(16.55 + 0.4, 3.682 - 0.4),
            new Translation2d(16.55 + 0.4, 4.962 + 0.4));
    public static final Zone BLUE_TOWER_ZONE = new Zone(
            new Translation2d(0.0 - 0.4, 4.351 + 0.4),
            new Translation2d(0.0 - 0.4, 3.185 - 0.4),
            new Translation2d(1.03 + 0.4, 3.185 - 0.4),
            new Translation2d(1.03 + 0.4, 4.351 + 0.4));
}
