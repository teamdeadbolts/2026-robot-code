/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

public class HopperData implements StructSerializable {
    public double targetHeight;
    public double leftCurrentHeight;
    public double leftOutput;
    public double leftRawMotorPosition;

    public double rightCurrentHeight;
    public double rightOutput;
    public double rightRawMotorPosition;

    public double debugCurrentHopperLeft;
    public double debugCurrentHopperRight;

    public HopperData() {}

    public HopperData(
            double targetHeight,
            double leftCurrentHeight,
            double leftOutput,
            double leftRawMotorPosition,
            double rightCurrentHeight,
            double rightOutput,
            double rightRawMotorPosition,
            double debugCurrentHopperLeft,
            double debugCurrentHopperRight) {
        this.targetHeight = targetHeight;
        this.leftCurrentHeight = leftCurrentHeight;
        this.leftOutput = leftOutput;
        this.leftRawMotorPosition = leftRawMotorPosition;
        this.rightCurrentHeight = rightCurrentHeight;
        this.rightOutput = rightOutput;
        this.rightRawMotorPosition = rightRawMotorPosition;
        this.debugCurrentHopperLeft = debugCurrentHopperLeft;
        this.debugCurrentHopperRight = debugCurrentHopperRight;
    }

    @Override
    public String toString() {
        return "HopperData{" + "targetHeight="
                + targetHeight + ", leftCurrentHeight="
                + leftCurrentHeight + ", leftOutput="
                + leftOutput + ", leftRawMotorPosition="
                + leftRawMotorPosition + ", rightCurrentHeight="
                + rightCurrentHeight + ", rightOutput="
                + rightOutput + ", rightRawMotorPosition="
                + rightRawMotorPosition + ", debugCurrentHopperLeft="
                + debugCurrentHopperLeft + ", debugCurrentHopperRight="
                + debugCurrentHopperRight + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof HopperData that)) return false;
        return Double.compare(targetHeight, that.targetHeight) == 0
                && Double.compare(leftCurrentHeight, that.leftCurrentHeight) == 0
                && Double.compare(leftOutput, that.leftOutput) == 0
                && Double.compare(leftRawMotorPosition, that.leftRawMotorPosition) == 0
                && Double.compare(rightCurrentHeight, that.rightCurrentHeight) == 0
                && Double.compare(rightOutput, that.rightOutput) == 0
                && Double.compare(rightRawMotorPosition, that.rightRawMotorPosition) == 0
                && Double.compare(debugCurrentHopperLeft, that.debugCurrentHopperLeft) == 0
                && Double.compare(debugCurrentHopperRight, that.debugCurrentHopperRight) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                targetHeight,
                leftCurrentHeight,
                leftOutput,
                leftRawMotorPosition,
                rightCurrentHeight,
                rightOutput,
                rightRawMotorPosition,
                debugCurrentHopperLeft,
                debugCurrentHopperRight);
    }

    public static final HopperDataStruct struct = new HopperDataStruct();
}
