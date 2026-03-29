/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class DebugSwerveDataStruct implements Struct<DebugSwerveData> {
    @Override
    public Class<DebugSwerveData> getTypeClass() {
        return DebugSwerveData.class;
    }

    @Override
    public String getTypeName() {
        return "DebugSwerveData";
    }

    @Override
    public int getSize() {
        return kSizeDouble * 2;
    }

    @Override
    public String getSchema() {
        return "double drive;double turn;";
    }

    @Override
    public DebugSwerveData unpack(ByteBuffer bb) {
        return new DebugSwerveData(bb.getDouble(), bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, DebugSwerveData value) {
        bb.putDouble(value.drive);
        bb.putDouble(value.turn);
    }
}
