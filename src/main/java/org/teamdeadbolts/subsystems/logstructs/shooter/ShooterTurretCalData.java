/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs.shooter;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterTurretCalData(
        double rawTargetAngle, double targetEncoderDeg, double shortestPathError, String limitHit)
        implements StructSerializable {
    public static final Struct<ShooterTurretCalData> struct = StructGenerator.genRecord(ShooterTurretCalData.class);
}
