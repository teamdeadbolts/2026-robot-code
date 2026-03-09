/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Vision related constants
 */
public class VisionConstants {
    // The layout of the april tags on the field
    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // The transform from the center of the turret to the on-turret camera
    public static final Transform3d TURRET_CAM_TO_TURRET = new Transform3d();

    // 2.1971
    // 1.7526 -.968
    public static final Transform3d LEFT_CAM_TRANSFORM = new Transform3d(
            new Translation3d(-0.104, 0.325 + 0.5851, 0.193),
            new Rotation3d(0, Units.degreesToRadians(25), Math.PI / 2));
    public static final Transform3d RIGHT_CAM_TRANSFORM = new Transform3d(
            new Translation3d(-0.1651, -.7846 - 0.19795, 0.0),
            new Rotation3d(0, Units.degreesToRadians(25), -Math.PI / 2));
}
