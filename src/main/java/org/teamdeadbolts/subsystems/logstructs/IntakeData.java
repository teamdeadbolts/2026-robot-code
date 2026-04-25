/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

public class IntakeData implements StructSerializable {
    public boolean armAtGoal;
    public double armSetpointPos;
    public double armSetpointVel;
    public double armPidPosError;
    public double armCurrVelocity;
    public double armTargetAngle;
    public double armTrapizoidSetpointPos;
    public double armTrapizoidSetpointVel;
    public double armEffortPIDVolts;
    public double armEffortFFVolts;
    public double armEffortObserverVolts;
    public double armCurrentAngle;
    public double armOutputVolts;
    public double wheelsVoltage;

    public IntakeData() {}

    public IntakeData(
            boolean armAtGoal,
            double armSetpointPos,
            double armSetpointVel,
            double armPidPosError,
            double armCurrVelocity,
            double armTargetAngle,
            double armTrapizoidSetpointPos,
            double armTrapizoidSetpointVel,
            double armEffortPIDVolts,
            double armEffortFFVolts,
            double armEffortObserverVolts,
            double armCurrentAngle,
            double armOutputVolts,
            double wheelsVoltage) {
        this.armAtGoal = armAtGoal;
        this.armSetpointPos = armSetpointPos;
        this.armSetpointVel = armSetpointVel;
        this.armPidPosError = armPidPosError;
        this.armCurrVelocity = armCurrVelocity;
        this.armTargetAngle = armTargetAngle;
        this.armTrapizoidSetpointPos = armTrapizoidSetpointPos;
        this.armTrapizoidSetpointVel = armTrapizoidSetpointVel;
        this.armEffortPIDVolts = armEffortPIDVolts;
        this.armEffortFFVolts = armEffortFFVolts;
        this.armEffortObserverVolts = armEffortObserverVolts;
        this.armCurrentAngle = armCurrentAngle;
        this.armOutputVolts = armOutputVolts;
        this.wheelsVoltage = wheelsVoltage;
    }

    @Override
    public String toString() {
        return "IntakeData{" + "armAtGoal="
                + armAtGoal + ", armSetpointPos="
                + armSetpointPos + ", armSetpointVel="
                + armSetpointVel + ", armPidPosError="
                + armPidPosError + ", armCurrVelocity="
                + armCurrVelocity + ", armTargetAngle="
                + armTargetAngle + ", armTrapizoidSetpointPos="
                + armTrapizoidSetpointPos + ", armTrapizoidSetpointVel="
                + armTrapizoidSetpointVel + ", armEffortPIDVolts="
                + armEffortPIDVolts + ", armEffortFFVolts="
                + armEffortFFVolts + ", armEffortObserverVolts="
                + armEffortObserverVolts + ", armCurrentAngle="
                + armCurrentAngle + ", armOutputVolts="
                + armOutputVolts + ", wheelsVoltage="
                + wheelsVoltage + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof IntakeData that)) return false;
        return armAtGoal == that.armAtGoal
                && Double.compare(armSetpointPos, that.armSetpointPos) == 0
                && Double.compare(armSetpointVel, that.armSetpointVel) == 0
                && Double.compare(armPidPosError, that.armPidPosError) == 0
                && Double.compare(armCurrVelocity, that.armCurrVelocity) == 0
                && Double.compare(armTargetAngle, that.armTargetAngle) == 0
                && Double.compare(armTrapizoidSetpointPos, that.armTrapizoidSetpointPos) == 0
                && Double.compare(armTrapizoidSetpointVel, that.armTrapizoidSetpointVel) == 0
                && Double.compare(armEffortPIDVolts, that.armEffortPIDVolts) == 0
                && Double.compare(armEffortFFVolts, that.armEffortFFVolts) == 0
                && Double.compare(armEffortObserverVolts, that.armEffortObserverVolts) == 0
                && Double.compare(armCurrentAngle, that.armCurrentAngle) == 0
                && Double.compare(armOutputVolts, that.armOutputVolts) == 0
                && Double.compare(wheelsVoltage, that.wheelsVoltage) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(
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

    public static final IntakeDataStruct struct = new IntakeDataStruct();
}
