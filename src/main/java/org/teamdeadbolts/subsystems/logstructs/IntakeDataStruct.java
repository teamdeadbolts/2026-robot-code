/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class IntakeDataStruct implements Struct<IntakeData> {
    @Override
    public Class<IntakeData> getTypeClass() {
        return IntakeData.class;
    }

    @Override
    public String getTypeName() {
        return "IntakeData";
    }

    @Override
    public int getSize() {
        return kSizeBool + kSizeDouble * 13;
    }

    @Override
    public String getSchema() {
        return "bool armAtGoal;"
                + "double armSetpointPos;"
                + "double armSetpointVel;"
                + "double armPidPosError;"
                + "double armCurrVelocity;"
                + "double armTargetAngle;"
                + "double armTrapizoidSetpointPos;"
                + "double armTrapizoidSetpointVel;"
                + "double armEffortPIDVolts;"
                + "double armEffortFFVolts;"
                + "double armEffortObserverVolts;"
                + "double armCurrentAngle;"
                + "double wheelsVoltage;";
    }

    @Override
    public IntakeData unpack(ByteBuffer bb) {
        boolean armAtGoal = bb.get() != 0;
        double armSetpointPos = bb.getDouble();
        double armSetpointVel = bb.getDouble();
        double armPidPosError = bb.getDouble();
        double armCurrVelocity = bb.getDouble();
        double armTargetAngle = bb.getDouble();
        double armTrapizoidSetpointPos = bb.getDouble();
        double armTrapizoidSetpointVel = bb.getDouble();
        double armEffortPIDVolts = bb.getDouble();
        double armEffortFFVolts = bb.getDouble();
        double armEffortObserverVolts = bb.getDouble();
        double armCurrentAngle = bb.getDouble();
        double armOutputVolts = bb.getDouble();
        double wheelsVoltage = bb.getDouble();
        return new IntakeData(
                armAtGoal,
                armSetpointPos,
                armSetpointVel,
                armPidPosError,
                armCurrVelocity,
                armTargetAngle,
                armTrapizoidSetpointPos,
                armTrapizoidSetpointVel,
                armEffortPIDVolts,
                armEffortFFVolts,
                armEffortObserverVolts,
                armCurrentAngle,
                armOutputVolts,
                wheelsVoltage);
    }

    @Override
    public void pack(ByteBuffer bb, IntakeData value) {
        bb.put((byte) (value.armAtGoal ? 1 : 0));
        bb.putDouble(value.armSetpointPos);
        bb.putDouble(value.armSetpointVel);
        bb.putDouble(value.armPidPosError);
        bb.putDouble(value.armCurrVelocity);
        bb.putDouble(value.armTargetAngle);
        bb.putDouble(value.armTrapizoidSetpointPos);
        bb.putDouble(value.armTrapizoidSetpointVel);
        bb.putDouble(value.armEffortPIDVolts);
        bb.putDouble(value.armEffortFFVolts);
        bb.putDouble(value.armEffortObserverVolts);
        bb.putDouble(value.armCurrentAngle);
        bb.putDouble(value.armOutputVolts);
        bb.putDouble(value.wheelsVoltage);
    }
}
