/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

public class ShooterData implements StructSerializable {
    public Translation2d aprilTagTrackRangeVertex;
    public Translation2d aprilTagTrackRangeForward;
    public Translation2d aprilTagTrackRangeSide;
    public Pose3d aprilTagTrackTargetTagPose;

    public Pose2d passTargetPose;

    public boolean fallBackFallBack;
    public Rotation2d fallbackChassisTarget;

    public double turretNormalizedSetpoint;
    public Pose2d targetTurretPose;
    public double turretPidOutput;
    public double turretPidError;
    public double turretTrapezoidSetpointPosDeg;
    public double turretTrapezoidSetpointVelDeg;

    // Hood
    public double hoodTargetHoodAngle;
    public double hoodHoodOutput;
    public double hoodCurrentAngle;
    public double hoodOutAmps;
    public boolean hoodUseAlternativeMinAngle;

    // Turret
    public Pose3d turretPose;
    public double turretCurrentPosition;
    public double turretVelocityDegPerSec;

    // Calculate  turret setpoint
    public int turretLimitHit; // 2 = MAX, 1 = MIN, 0 = NONE, -1 = NOT SET
    public double turretRawTargetAngle;
    public double turretTargetEncoderDeg;
    public double turretShortestPathError;

    // Wheels
    public double wheelVolts;
    public double wheelTargetSpeedRPM;
    public double wheelRPMError;
    public double wheelCurrentSpeed;
    public double wheelLeftMotorVolts;
    public double wheelRightMotorVolts;

    // Current
    public double debugCurrentShooterHood;
    public double debugCurrentShooterTurret;
    public double debugCurrentShooterLeftWheel;

    public ShooterData() {
        aprilTagTrackRangeVertex = Translation2d.kZero;
        aprilTagTrackRangeForward = Translation2d.kZero;
        aprilTagTrackRangeSide = Translation2d.kZero;
        aprilTagTrackTargetTagPose = Pose3d.kZero;
        passTargetPose = Pose2d.kZero;
        fallbackChassisTarget = Rotation2d.kZero;
        targetTurretPose = Pose2d.kZero;
        turretPose = Pose3d.kZero;
        turretLimitHit = -1;
    }

    public ShooterData(
            Translation2d aprilTagTrackRangeVertex,
            Translation2d aprilTagTrackRangeForward,
            Translation2d aprilTagTrackRangeSide,
            Pose3d aprilTagTrackTargetTagPose,
            Pose2d passTargetPose,
            boolean fallBackFallBack,
            Rotation2d fallbackChassisTarget,
            double turretNormalizedSetpoint,
            Pose2d targetTurretPose,
            double turretPidOutput,
            double turretPidError,
            double turretTrapezoidSetpointPosDeg,
            double turretTrapezoidSetpointVelDeg,
            double hoodTargetHoodAngle,
            double hoodHoodOutput,
            double hoodCurrentAngle,
            double hoodOutAmps,
            boolean hoodUseAlternativeMinAngle,
            Pose3d turretPose,
            double turretCurrentPosition,
            double turretVelocityDegPerSec,
            int turretLimitHit,
            double turretRawTargetAngle,
            double turretTargetEncoderDeg,
            double turretShortestPathError,
            double wheelVolts,
            double wheelTargetSpeedRPM,
            double wheelRPMError,
            double wheelCurrentSpeed,
            double wheelLeftMotorVolts,
            double wheelRightMotorVolts,
            double debugCurrentShooterHood,
            double debugCurrentShooterTurret,
            double debugCurrentShooterLeftWheel) {
        this.aprilTagTrackRangeVertex = aprilTagTrackRangeVertex;
        this.aprilTagTrackRangeForward = aprilTagTrackRangeForward;
        this.aprilTagTrackRangeSide = aprilTagTrackRangeSide;
        this.aprilTagTrackTargetTagPose = aprilTagTrackTargetTagPose;
        this.passTargetPose = passTargetPose;
        this.fallBackFallBack = fallBackFallBack;
        this.fallbackChassisTarget = fallbackChassisTarget;
        this.turretNormalizedSetpoint = turretNormalizedSetpoint;
        this.targetTurretPose = targetTurretPose;
        this.turretPidOutput = turretPidOutput;
        this.turretPidError = turretPidError;
        this.turretTrapezoidSetpointPosDeg = turretTrapezoidSetpointPosDeg;
        this.turretTrapezoidSetpointVelDeg = turretTrapezoidSetpointVelDeg;
        this.hoodTargetHoodAngle = hoodTargetHoodAngle;
        this.hoodHoodOutput = hoodHoodOutput;
        this.hoodCurrentAngle = hoodCurrentAngle;
        this.hoodOutAmps = hoodOutAmps;
        this.hoodUseAlternativeMinAngle = hoodUseAlternativeMinAngle;
        this.turretPose = turretPose;
        this.turretCurrentPosition = turretCurrentPosition;
        this.turretVelocityDegPerSec = turretVelocityDegPerSec;
        this.turretLimitHit = turretLimitHit;
        this.turretRawTargetAngle = turretRawTargetAngle;
        this.turretTargetEncoderDeg = turretTargetEncoderDeg;
        this.turretShortestPathError = turretShortestPathError;
        this.wheelVolts = wheelVolts;
        this.wheelTargetSpeedRPM = wheelTargetSpeedRPM;
        this.wheelRPMError = wheelRPMError;
        this.wheelCurrentSpeed = wheelCurrentSpeed;
        this.wheelLeftMotorVolts = wheelLeftMotorVolts;
        this.wheelRightMotorVolts = wheelRightMotorVolts;
        this.debugCurrentShooterHood = debugCurrentShooterHood;
        this.debugCurrentShooterTurret = debugCurrentShooterTurret;
        this.debugCurrentShooterLeftWheel = debugCurrentShooterLeftWheel;
    }

    @Override
    public String toString() {
        return "ShooterData{" + "aprilTagTrackRangeVertex="
                + aprilTagTrackRangeVertex + ", aprilTagTrackRangeForward="
                + aprilTagTrackRangeForward + ", aprilTagTrackRangeSide="
                + aprilTagTrackRangeSide + ", aprilTagTrackTargetTagPose="
                + aprilTagTrackTargetTagPose + ", passTargetPose="
                + passTargetPose + ", fallBackFallBack="
                + fallBackFallBack + ", fallbackChassisTarget="
                + fallbackChassisTarget + ", turretNormalizedSetpoint="
                + turretNormalizedSetpoint + ", targetTurretPose="
                + targetTurretPose + ", turretPidOutput="
                + turretPidOutput + ", turretPidError="
                + turretPidError + ", turretTrapezoidSetpointPosDeg="
                + turretTrapezoidSetpointPosDeg + ", turretTrapezoidSetpointVelDeg="
                + turretTrapezoidSetpointVelDeg + ", hoodTargetHoodAngle="
                + hoodTargetHoodAngle + ", hoodHoodOutput="
                + hoodHoodOutput + ", hoodCurrentAngle="
                + hoodCurrentAngle + ", hoodOutAmps="
                + hoodOutAmps + ", hoodUseAlternativeMinAngle="
                + hoodUseAlternativeMinAngle + ", turretPose="
                + turretPose + ", turretCurrentPosition="
                + turretCurrentPosition + ", turretVelocityDegPerSec="
                + turretVelocityDegPerSec + ", turretLimitHit='"
                + turretLimitHit + '\'' + ", turretRawTargetAngle="
                + turretRawTargetAngle + ", turretTargetEncoderDeg="
                + turretTargetEncoderDeg + ", turretShortestPathError="
                + turretShortestPathError + ", wheelVolts="
                + wheelVolts + ", wheelTargetSpeedRPM="
                + wheelTargetSpeedRPM + ", wheelRPMError="
                + wheelRPMError + ", wheelCurrentSpeed="
                + wheelCurrentSpeed + ", wheelLeftMotorVolts="
                + wheelLeftMotorVolts + ", wheelRightMotorVolts="
                + wheelRightMotorVolts + ", debugCurrentShooterHood="
                + debugCurrentShooterHood + ", debugCurrentShooterTurret="
                + debugCurrentShooterTurret + ", debugCurrentShooterLeftWheel="
                + debugCurrentShooterLeftWheel + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof ShooterData that)) return false;
        return fallBackFallBack == that.fallBackFallBack
                && Double.compare(turretNormalizedSetpoint, that.turretNormalizedSetpoint) == 0
                && Double.compare(turretPidOutput, that.turretPidOutput) == 0
                && Double.compare(turretPidError, that.turretPidError) == 0
                && Double.compare(turretTrapezoidSetpointPosDeg, that.turretTrapezoidSetpointPosDeg) == 0
                && Double.compare(turretTrapezoidSetpointVelDeg, that.turretTrapezoidSetpointVelDeg) == 0
                && Double.compare(hoodTargetHoodAngle, that.hoodTargetHoodAngle) == 0
                && Double.compare(hoodHoodOutput, that.hoodHoodOutput) == 0
                && Double.compare(hoodCurrentAngle, that.hoodCurrentAngle) == 0
                && Double.compare(hoodOutAmps, that.hoodOutAmps) == 0
                && Boolean.compare(hoodUseAlternativeMinAngle, that.hoodUseAlternativeMinAngle) == 0
                && Double.compare(turretCurrentPosition, that.turretCurrentPosition) == 0
                && Double.compare(turretVelocityDegPerSec, that.turretVelocityDegPerSec) == 0
                && turretLimitHit == that.turretLimitHit
                && Double.compare(turretRawTargetAngle, that.turretRawTargetAngle) == 0
                && Double.compare(turretTargetEncoderDeg, that.turretTargetEncoderDeg) == 0
                && Double.compare(turretShortestPathError, that.turretShortestPathError) == 0
                && Double.compare(wheelVolts, that.wheelVolts) == 0
                && Double.compare(wheelTargetSpeedRPM, that.wheelTargetSpeedRPM) == 0
                && Double.compare(wheelRPMError, that.wheelRPMError) == 0
                && Double.compare(wheelCurrentSpeed, that.wheelCurrentSpeed) == 0
                && Double.compare(wheelLeftMotorVolts, that.wheelLeftMotorVolts) == 0
                && Double.compare(wheelRightMotorVolts, that.wheelRightMotorVolts) == 0
                && Double.compare(debugCurrentShooterHood, that.debugCurrentShooterHood) == 0
                && Double.compare(debugCurrentShooterTurret, that.debugCurrentShooterTurret) == 0
                && Double.compare(debugCurrentShooterLeftWheel, that.debugCurrentShooterLeftWheel) == 0
                && Objects.equals(aprilTagTrackRangeVertex, that.aprilTagTrackRangeVertex)
                && Objects.equals(aprilTagTrackRangeForward, that.aprilTagTrackRangeForward)
                && Objects.equals(aprilTagTrackRangeSide, that.aprilTagTrackRangeSide)
                && Objects.equals(aprilTagTrackTargetTagPose, that.aprilTagTrackTargetTagPose)
                && Objects.equals(passTargetPose, that.passTargetPose)
                && Objects.equals(fallbackChassisTarget, that.fallbackChassisTarget)
                && Objects.equals(targetTurretPose, that.targetTurretPose)
                && Objects.equals(turretPose, that.turretPose);
    }

    @Override
    public int hashCode() {
        return Objects.hash(
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

    public static final ShooterDataStruct struct = new ShooterDataStruct();
}
