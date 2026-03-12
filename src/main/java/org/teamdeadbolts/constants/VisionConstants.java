/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

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
    public static final Transform3d LEFT_CAM_TRANSFORM = new Transform3d();
    public static final Transform3d RIGHT_CAM_TRANSFORM = new Transform3d();
    public static final Transform3d BACK_CAM_TRANSFORM = new Transform3d();
}
