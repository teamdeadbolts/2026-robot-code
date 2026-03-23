/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

public class DebugSwerveData implements StructSerializable {
    public double drive;
    public double turn;

    public DebugSwerveData() {}

    public DebugSwerveData(double drive, double turn) {
        this.drive = drive;
        this.turn = turn;
    }

    @Override
    public String toString() {
        return "DebugSwerveData{" + "drive=" + drive + ", turn=" + turn + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof DebugSwerveData that)) return false;
        return Double.compare(drive, that.drive) == 0 && Double.compare(turn, that.turn) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(drive, turn);
    }

    public static final DebugSwerveDataStruct struct = new DebugSwerveDataStruct();
}
