/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterTurretData(
        double normalizedSetpoint,
        Pose2d turretPose,
        double pidOutput,
        double pidError,
        double trapezoidSetpointPosDeg,
        double trapezoidSetpointVelDeg)
        implements StructSerializable {
    public static final Struct<ShooterTurretData> struct = StructGenerator.genRecord(ShooterTurretData.class);
}
