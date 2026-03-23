/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterSummaryData(
        double hoodCurrentAngle,
        double hoodOutAmps,
        boolean hoodUseAlternativeMinAngle,
        Pose3d turretPose,
        double turretCurrentPosition,
        double turretVelocityDegPerSec,
        double wheelCurrentSpeed,
        double wheelLeftMotorVolts,
        double wheelRightMotorVolts)
        implements StructSerializable {
    public static final Struct<ShooterSummaryData> struct = StructGenerator.genRecord(ShooterSummaryData.class);
}
