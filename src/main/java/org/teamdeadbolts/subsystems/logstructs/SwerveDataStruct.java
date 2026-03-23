/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.logstructs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class SwerveDataStruct implements Struct<SwerveData> {

    @Override
    public Class<SwerveData> getTypeClass() {
        return SwerveData.class;
    }

    @Override
    public String getTypeName() {
        return "SwerveData";
    }

    @Override
    public int getSize() {
        return kSizeDouble * 11;
    }

    @Override
    public String getSchema() {
        return "double driveCurrent;"
                + "double turnCurrent;"
                + "double drivePIDOut;"
                + "double driveVoltageOut;"
                + "double driveReportedMPS;"
                + "double driveReportedRPS;"
                + "double driveTargetMPS;"
                + "double drivePIDError;"
                + "double turnPIDError;"
                + "double turnPIDSetpoint;"
                + "double turnMeasurementDeg;";
    }

    @Override
    public SwerveData unpack(ByteBuffer bb) {
        double driveCurrent = bb.getDouble();
        double turnCurrent = bb.getDouble();
        double drivePIDOut = bb.getDouble();
        double driveVoltageOut = bb.getDouble();
        double driveReportedMPS = bb.getDouble();
        double driveReportedRPS = bb.getDouble();
        double driveTargetMPS = bb.getDouble();
        double drivePIDError = bb.getDouble();
        double turnPIDError = bb.getDouble();
        double turnPIDSetpoint = bb.getDouble();
        double turnMeasurementDeg = bb.getDouble();
        return new SwerveData(
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

    @Override
    public void pack(ByteBuffer bb, SwerveData value) {
        bb.putDouble(value.driveCurrent);
        bb.putDouble(value.turnCurrent);
        bb.putDouble(value.drivePIDOut);
        bb.putDouble(value.driveVoltageOut);
        bb.putDouble(value.driveReportedMPS);
        bb.putDouble(value.driveReportedRPS);
        bb.putDouble(value.driveTargetMPS);
        bb.putDouble(value.drivePIDError);
        bb.putDouble(value.turnPIDError);
        bb.putDouble(value.turnPIDSetpoint);
        bb.putDouble(value.turnMeasurementDeg);
    }
}
