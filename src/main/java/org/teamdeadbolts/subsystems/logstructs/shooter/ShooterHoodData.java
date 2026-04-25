/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs.shooter;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterHoodData(double targetHoodAngle, double pidOutput) implements StructSerializable {
    public static final Struct<ShooterHoodData> struct = StructGenerator.genRecord(ShooterHoodData.class);
}
