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
            new Transform3d(new Translation3d(-0.386, -0.212, 0.136), new Rotation3d(0, -0.509, 3.114));
    public static final Transform3d RIGHT_CAM_TRANSFORM =
            new Transform3d(new Translation3d(-0.210, -0.364, 0.154), new Rotation3d(0, -0.500, -1.602));
    public static final Transform3d LEFT_CAM_TRANSFORM =
            new Transform3d(new Translation3d(-0.216, 0.184, 0.112), new Rotation3d(0, -0.445, 1.619));
}
