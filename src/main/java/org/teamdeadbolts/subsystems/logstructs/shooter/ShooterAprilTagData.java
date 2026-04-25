/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterAprilTagData(
        Translation2d trackRangeVertex,
        Translation2d trackRangeForward,
        Translation2d trackRangeSide,
        Pose3d trackTargetTagPose)
        implements StructSerializable {

    public static final Struct<ShooterAprilTagData> struct = StructGenerator.genRecord(ShooterAprilTagData.class);
}
