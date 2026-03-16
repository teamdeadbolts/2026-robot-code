/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class ZoneTest {
    @Test
    public void testIsInZoneSquare() {
        Translation2d topLeft = new Translation2d(0, 1);
        Translation2d topRight = new Translation2d(1, 1);
        Translation2d bottomLeft = new Translation2d(0, 0);
        Translation2d bottomRight = new Translation2d(1, 0);

        Zone zone = new Zone(topLeft, topRight, bottomRight, bottomLeft);

        assertTrue(zone.contains(new Translation2d(0.5, 0.5)));
        assertTrue(zone.contains(new Translation2d(0.25, 0.9)));
        assertFalse(zone.contains(new Translation2d(2, 2)));
        assertFalse(zone.contains(new Translation2d(-1, -1)));
    }

    @Test
    public void testZoneInPolygon() {
        Translation2d[] vertices = {
            new Translation2d(0, 0),
            new Translation2d(3, 0),
            new Translation2d(5, 2),
            new Translation2d(2, 4),
            new Translation2d(4, 7),
            new Translation2d(0, 5),
            new Translation2d(-2, 2),
        };

        Zone zone = new Zone(vertices);

        assertTrue(zone.contains(new Translation2d(1, 1)));
        assertTrue(zone.contains(new Translation2d(2, 5)));
        assertFalse(zone.contains(new Translation2d(-1, -1)));
        assertFalse(zone.contains(new Translation2d(10, 10)));
    }
}
