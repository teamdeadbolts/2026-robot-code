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
            new Transform3d(new Translation3d(-0.108, -0.053, 0.201), new Rotation3d(0.0, -0.376, 0));

    // 2.1971
    // 1.7526 -.968
    public static final Transform3d BACK_CAM_TRANSFORM =
            new Transform3d(new Translation3d(-0.158, -0.239, 0.146), new Rotation3d(0, -0.435, 3.137));
    public static final Transform3d RIGHT_CAM_TRANSFORM =
            new Transform3d(new Translation3d(-0.082, -0.427, 0.371), new Rotation3d(0, -0.312, -1.693));
    public static final Transform3d LEFT_CAM_TRANSFORM =
            new Transform3d(new Translation3d(0.017, 0.187, 0.070), new Rotation3d(0, -0.489, 1.715));
}
