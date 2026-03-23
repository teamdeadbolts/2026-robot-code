/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs.shooter;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterWheelData(double wheelOutput, double wheelTargetSpeedRPM, double wheelRPMError)
        implements StructSerializable {
    public static final Struct<ShooterWheelData> struct = StructGenerator.genRecord(ShooterWheelData.class);
}
