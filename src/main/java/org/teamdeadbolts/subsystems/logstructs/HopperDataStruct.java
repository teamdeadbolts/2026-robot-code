/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class HopperDataStruct implements Struct<HopperData> {
    @Override
    public Class<HopperData> getTypeClass() {
        return HopperData.class;
    }

    @Override
    public String getTypeName() {
        return "HopperData";
    }

    @Override
    public int getSize() {
        return 9 * kSizeDouble;
    }

    @Override
    public String getSchema() {
        return "double targetHeight;" + "    double leftCurrentHeight;"
                + "    double leftOutput;"
                + "    double leftRawMotorPosition;"
                + "    double rightCurrentHeight;"
                + "    double rightOutput;"
                + "    double rightRawMotorPosition;"
                + "    double debugCurrentHopperLeft;"
                + "    double debugCurrentHopperRight;";
    }

    @Override
    public HopperData unpack(ByteBuffer bb) {
        double targetHeight = bb.getDouble();
        double leftCurrentHeight = bb.getDouble();
        double leftOutput = bb.getDouble();
        double leftRawMotorPosition = bb.getDouble();

        double rightCurrentHeight = bb.getDouble();
        double rightOutput = bb.getDouble();
        double rightRawMotorPosition = bb.getDouble();

        double debugCurrentHopperLeft = bb.getDouble();
        double debugCurrentHopperRight = bb.getDouble();
        return new HopperData(
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

    @Override
    public void pack(ByteBuffer bb, HopperData value) {
        bb.putDouble(value.targetHeight);
        bb.putDouble(value.leftCurrentHeight);
        bb.putDouble(value.leftOutput);
        bb.putDouble(value.leftRawMotorPosition);
        bb.putDouble(value.rightCurrentHeight);
        bb.putDouble(value.rightOutput);
        bb.putDouble(value.rightRawMotorPosition);
        bb.putDouble(value.debugCurrentHopperLeft);
        bb.putDouble(value.debugCurrentHopperRight);
    }
}
