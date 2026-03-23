/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.StructSerializable;
import java.util.Objects;

public class SwerveData implements StructSerializable {
    public double driveCurrent;
    public double turnCurrent;
    public double drivePIDOut;
    public double driveVoltageOut;
    public double driveReportedMPS;
    public double driveReportedRPS;
    public double driveTargetMPS;
    public double drivePIDError;
    public double turnPIDError;
    public double turnPIDSetpoint;
    public double turnMeasurementDeg;

    public SwerveData() {}

    public SwerveData(
            double driveCurrent,
            double turnCurrent,
            double drivePIDOut,
            double driveVoltageOut,
            double driveReportedMPS,
            double driveReportedRPS,
            double driveTargetMPS,
            double drivePIDError,
            double turnPIDError,
            double turnPIDSetpoint,
            double turnMeasurementDeg) {
        this.driveCurrent = driveCurrent;
        this.turnCurrent = turnCurrent;
        this.drivePIDOut = drivePIDOut;
        this.driveVoltageOut = driveVoltageOut;
        this.driveReportedMPS = driveReportedMPS;
        this.driveReportedRPS = driveReportedRPS;
        this.driveTargetMPS = driveTargetMPS;
        this.drivePIDError = drivePIDError;
        this.turnPIDError = turnPIDError;
        this.turnPIDSetpoint = turnPIDSetpoint;
        this.turnMeasurementDeg = turnMeasurementDeg;
    }

    @Override
    public String toString() {
        return "SwerveData{" + "driveCurrent="
                + driveCurrent + ", turnCurrent="
                + turnCurrent + ", drivePIDOut="
                + drivePIDOut + ", driveVoltageOut="
                + driveVoltageOut + ", driveReportedMPS="
                + driveReportedMPS + ", driveReportedRPS="
                + driveReportedRPS + ", driveTargetMPS="
                + driveTargetMPS + ", drivePIDError="
                + drivePIDError + ", turnPIDError="
                + turnPIDError + ", turnPIDSetpoint="
                + turnPIDSetpoint + ", turnMeasurementDeg="
                + turnMeasurementDeg + '}';
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof SwerveData that)) return false;
        return Double.compare(driveCurrent, that.driveCurrent) == 0
                && Double.compare(turnCurrent, that.turnCurrent) == 0
                && Double.compare(drivePIDOut, that.drivePIDOut) == 0
                && Double.compare(driveVoltageOut, that.driveVoltageOut) == 0
                && Double.compare(driveReportedMPS, that.driveReportedMPS) == 0
                && Double.compare(driveReportedRPS, that.driveReportedRPS) == 0
                && Double.compare(driveTargetMPS, that.driveTargetMPS) == 0
                && Double.compare(drivePIDError, that.drivePIDError) == 0
                && Double.compare(turnPIDError, that.turnPIDError) == 0
                && Double.compare(turnPIDSetpoint, that.turnPIDSetpoint) == 0
                && Double.compare(turnMeasurementDeg, that.turnMeasurementDeg) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                driveCurrent,
                turnCurrent,
                drivePIDOut,
                driveVoltageOut,
                driveReportedMPS,
                driveReportedRPS,
                driveTargetMPS,
                drivePIDError,
                turnPIDError,
                turnPIDSetpoint,
                turnMeasurementDeg);
    }

    public static final SwerveDataStruct struct = new SwerveDataStruct();
}
