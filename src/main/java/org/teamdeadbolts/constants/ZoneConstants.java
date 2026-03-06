/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import edu.wpi.first.math.geometry.Translation2d;
import org.teamdeadbolts.utils.Zone;

public class ZoneConstants {
    // TODO: Find these
    public static final Zone BLUE_TOP_BUMP_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));
    public static final Zone BLUE_BOTTOM_BUMP_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));
    public static final Zone BLUE_BOTTOM_TRENCH_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));
    public static final Zone BLUE_TOP_TRENCH_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));

    public static final Zone RED_TOP_BUMP_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));
    public static final Zone RED_BOTTOM_BUMP_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));
    public static final Zone RED_BOTTOM_TRENCH_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));
    public static final Zone RED_TOP_TRENCH_ZONE = new Zone(new Translation2d(, ), new Translation2d(, ), new Translation2d(, ), new Translation2d(, ));

    public static final Zone RED_SCORE_ZONE = new Zone(new Translation2d(11.97, 7.939), new Translation2d(11.97, 0.16), new Translation2d(16.45, 0.16), new Translation2d(16.45, 7.939));
    public static final Zone BLUE_SCORE_ZONE = new Zone(new Translation2d(0.0, 7.95), new Translation2d(0.0, 0.0), new Translation2d(4.4095, 0.0), new Translation2d(4.4095, 7.95));

    public static final Zone RED_HUB_CLEARANCE_ZONE = new Zone(new Translation2d(11.342, 4.617), new Translation2d(11.33, 3.455), new Translation2d(12.514+0.67, 3.467), new Translation2d(12.514+0.67, 4.633));
    public static final Zone BLUE_HUB_CLEARANCE_ZONE = new Zone(new Translation2d(4.0121-0.67056, 4.60), new Translation2d(4.0121-0.67056, 3.472), new Translation2d(5.212, 3.472), new Translation2d(5.212, 4.60));

    public static final Zone RED_TOWER_ZONE = new Zone(new Translation2d(15.455, 4.962), new Translation2d(15.455, 3.682), new Translation2d(16.55, 3.682), new Translation2d(16.55, 4.962));
    public static final Zone BLUE_TOWER_ZONE = new Zone(new Translation2d(0.0, 4.351), new Translation2d(0.0, 3.185), new Translation2d(1.03, 3.185), new Translation2d(1.03, 4.351));
}
