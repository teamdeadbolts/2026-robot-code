/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class ShooterDataStruct implements Struct<ShooterData> {
    @Override
    public Class<ShooterData> getTypeClass() {
        return ShooterData.class;
    }

    @Override
    public String getTypeName() {
        return "ShooterData";
    }

    @Override
    public int getSize() {
        return 3 * Translation2d.struct.getSize()
                + Pose3d.struct.getSize()
                + Pose2d.struct.getSize()
                + 2 * kSizeBool
                + Rotation2d.struct.getSize()
                + kSizeDouble
                + Pose2d.struct.getSize()
                + 4 * kSizeDouble
                + 4 * kSizeDouble
                + Pose3d.struct.getSize()
                + 2 * kSizeDouble
                + kSizeInt8
                + 3 * kSizeDouble
                + 9 * kSizeDouble;
    }

    @Override
    public String getSchema() {
        return "     Translation2d aprilTagTrackRangeVertex;" + "     Translation2d aprilTagTrackRangeForward;"
                + "     Translation2d aprilTagTrackRangeSide;"
                + "     Pose3d aprilTagTrackTargetTagPose;"
                + "     Pose2d passTargetPose;"
                + "     bool fallBackFallBack;"
                + "     Rotation2d fallbackChassisTarget;"
                + "     double turretNormalizedSetpoint;"
                + "     Pose2d targetTurretPose;"
                + "     double turretPidOutput;"
                + "     double turretPidError;"
                + "     double turretTrapezoidSetpointPosDeg;"
                + "     double turretTrapezoidSetpointVelDeg;"
                + "     double hoodTargetHoodAngle;"
                + "     double hoodHoodOutput;"
                + "     double hoodCurrentAngle;"
                + "     double hoodOutAmps;"
                + "     bool hoodUseAlternativeMinAngle;"
                + "     Pose3d turretPose;"
                + "     double turretCurrentPosition;"
                + "     double turretVelocityDegPerSec;"
                + "     int turretLimitHit;"
                + "     double turretRawTargetAngle;"
                + "     double turretTargetEncoderDeg;"
                + "     double turretShortestPathError;"
                + "     double wheelVolts;"
                + "     double wheelTargetSpeedRPM;"
                + "     double wheelRPMError;"
                + "     double wheelCurrentSpeed;"
                + "     double wheelLeftMotorVolts;"
                + "     double wheelRightMotorVolts;"
                + "     double debugCurrentShooterHood;"
                + "     double debugCurrentShooterTurret;"
                + "     double debugCurrentShooterLeftWheel;";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {Translation2d.struct, Rotation2d.struct, Pose2d.struct, Pose3d.struct};
    }

    @Override
    public ShooterData unpack(ByteBuffer bb) {
        Translation2d aprilTagTrackRangeVertex = Translation2d.struct.unpack(bb);
        Translation2d aprilTagTrackRangeForward = Translation2d.struct.unpack(bb);
        Translation2d aprilTagTrackRangeSide = Translation2d.struct.unpack(bb);
        Pose3d aprilTagTrackTargetTagPose = Pose3d.struct.unpack(bb);

        Pose2d passTargetPose = Pose2d.struct.unpack(bb);

        boolean fallBackFallBack = bb.get() != 0;
        Rotation2d fallbackChassisTarget = Rotation2d.struct.unpack(bb);

        double turretNormalizedSetpoint = bb.getDouble();
        Pose2d targetTurretPose = Pose2d.struct.unpack(bb);
        double turretPidOutput = bb.getDouble();
        double turretPidError = bb.getDouble();
        double turretTrapezoidSetpointPosDeg = bb.getDouble();
        double turretTrapezoidSetpointVelDeg = bb.getDouble();

        // Hood
        double hoodTargetHoodAngle = bb.getDouble();
        double hoodHoodOutput = bb.getDouble();
        double hoodCurrentAngle = bb.getDouble();
        double hoodOutAmps = bb.getDouble();
        boolean hoodUseAlternativeMinAngle = bb.get() != 0;

        // Turret
        Pose3d turretPose = Pose3d.struct.unpack(bb);
        double turretCurrentPosition = bb.getDouble();
        double turretVelocityDegPerSec = bb.getDouble();

        // Calculate  turret setpoint
        int turretLimitHit = bb.get(); // 2 = MAX, 1 = MIN, 0 = NONE
        double turretRawTargetAngle = bb.getDouble();
        double turretTargetEncoderDeg = bb.getDouble();
        double turretShortestPathError = bb.getDouble();

        // Wheels
        double wheelVolts = bb.getDouble();
        double wheelTargetSpeedRPM = bb.getDouble();
        double wheelRPMError = bb.getDouble();
        double wheelCurrentSpeed = bb.getDouble();
        double wheelLeftMotorVolts = bb.getDouble();
        double wheelRightMotorVolts = bb.getDouble();

        // Current
        double debugCurrentShooterHood = bb.getDouble();
        double debugCurrentShooterTurret = bb.getDouble();
        double debugCurrentShooterLeftWheel = bb.getDouble();

        return new ShooterData(
                aprilTagTrackRangeVertex,
                aprilTagTrackRangeForward,
                aprilTagTrackRangeSide,
                aprilTagTrackTargetTagPose,
                passTargetPose,
                fallBackFallBack,
                fallbackChassisTarget,
                turretNormalizedSetpoint,
                targetTurretPose,
                turretPidOutput,
                turretPidError,
                turretTrapezoidSetpointPosDeg,
                turretTrapezoidSetpointVelDeg,
                hoodTargetHoodAngle,
                hoodHoodOutput,
                hoodCurrentAngle,
                hoodOutAmps,
                hoodUseAlternativeMinAngle,
                turretPose,
                turretCurrentPosition,
                turretVelocityDegPerSec,
                turretLimitHit,
                turretRawTargetAngle,
                turretTargetEncoderDeg,
                turretShortestPathError,
                wheelVolts,
                wheelTargetSpeedRPM,
                wheelRPMError,
                wheelCurrentSpeed,
                wheelLeftMotorVolts,
                wheelRightMotorVolts,
                debugCurrentShooterHood,
                debugCurrentShooterTurret,
                debugCurrentShooterLeftWheel);
    }

    @Override
    public void pack(ByteBuffer bb, ShooterData value) {
        Translation2d.struct.pack(bb, value.aprilTagTrackRangeVertex);
        Translation2d.struct.pack(bb, value.aprilTagTrackRangeForward);
        Translation2d.struct.pack(bb, value.aprilTagTrackRangeSide);
        Pose3d.struct.pack(bb, value.aprilTagTrackTargetTagPose);

        Pose2d.struct.pack(bb, value.passTargetPose);
        bb.put((byte) (value.fallBackFallBack ? 1 : 0));
        Rotation2d.struct.pack(bb, value.fallbackChassisTarget);

        bb.putDouble(value.turretNormalizedSetpoint);
        Pose2d.struct.pack(bb, value.targetTurretPose);
        bb.putDouble(value.turretPidOutput);
        bb.putDouble(value.turretPidError);
        bb.putDouble(value.turretTrapezoidSetpointPosDeg);
        bb.putDouble(value.turretTrapezoidSetpointVelDeg);

        bb.putDouble(value.hoodTargetHoodAngle);
        bb.putDouble(value.hoodHoodOutput);
        bb.putDouble(value.hoodCurrentAngle);
        bb.putDouble(value.hoodOutAmps);
        bb.put((byte) (value.hoodUseAlternativeMinAngle ? 1 : 0));

        Pose3d.struct.pack(bb, value.turretPose);
        bb.putDouble(value.turretCurrentPosition);
        bb.putDouble(value.turretVelocityDegPerSec);

        bb.put((byte) value.turretLimitHit);
        bb.putDouble(value.turretRawTargetAngle);
        bb.putDouble(value.turretTargetEncoderDeg);
        bb.putDouble(value.turretShortestPathError);

        bb.putDouble(value.wheelVolts);
        bb.putDouble(value.wheelTargetSpeedRPM);
        bb.putDouble(value.wheelRPMError);
        bb.putDouble(value.wheelCurrentSpeed);
        bb.putDouble(value.wheelLeftMotorVolts);
        bb.putDouble(value.wheelRightMotorVolts);

        bb.putDouble(value.debugCurrentShooterHood);
        bb.putDouble(value.debugCurrentShooterTurret);
        bb.putDouble(value.debugCurrentShooterLeftWheel);
    }
}
