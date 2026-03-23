/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterFallbackData(boolean fallBack, Rotation2d chassisTarget) implements StructSerializable {
    public static final Struct<ShooterFallbackData> struct = StructGenerator.genRecord(ShooterFallbackData.class);
}
