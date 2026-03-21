/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Vision related constants
 */
public class VisionConstants {
    // The layout of the april tags on the field
    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // The transform from the center of the turret to the on-turret camera
    public static final Transform3d TURRET_CAM_TO_TURRET =
            new Transform3d(0.0773, 0.0843, 0.0731, new Rotation3d(0.0, -0.474, 3.118 - Math.PI));

    // 2.1971
    // 1.7526 -.968
    public static final Transform3d BACK_CAM_TRANSFORM =
            new Transform3d(new Translation3d(-0.414, -0.242, 0.111), new Rotation3d(0, -0.502, 3.110));
    public static final Transform3d RIGHT_CAM_TRANSFORM =
            new Transform3d(new Translation3d(-0.167, -0.357, 0.109), new Rotation3d(0, -0.494, -1.582));
    public static final Transform3d LEFT_CAM_TRANSFORM =
            new Transform3d(new Translation3d(-0.064, 0.407, 0.063), new Rotation3d(0, -0.502, 1.560));
}
