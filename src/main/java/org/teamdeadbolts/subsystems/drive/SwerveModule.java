/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.utils.MathUtils;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

/**
 * Provides hardware control for each swerve module
 */
public class SwerveModule implements Refreshable {
    private final int moduleNumber;
    private final Rotation2d offset;

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder encoder;

    private final CANBus canBus = new CANBus("*");

    /** Tuning values */
    private final SavedTunableNumber dFFkS = SavedTunableNumber.get("Tuning/Swerve/Drive/kS", 0.0);

    private final SavedTunableNumber dFFkV = SavedTunableNumber.get("Tuning/Swerve/Drive/kV", 0.0);
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(dFFkS.get(), dFFkS.get());

    private final SavedTunableNumber tFFkS = SavedTunableNumber.get("Tuning/Swerve/Turn/kS", 0.0);
    private final SavedTunableNumber tFFkV = SavedTunableNumber.get("Tuning/Swerve/Turn/kV", 0.0);
    private final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(tFFkS.get(), tFFkV.get());

    private final SavedTunableNumber dP = SavedTunableNumber.get("Tuning/Swerve/Drive/kP", 0.0);
    private final SavedTunableNumber dI = SavedTunableNumber.get("Tuning/Swerve/Drive/kI", 0.0);

    private final SavedTunableNumber tP = SavedTunableNumber.get("Tuning/Swerve/Turn/kP", 0.0);
    private final SavedTunableNumber tI = SavedTunableNumber.get("Tuning/Swerve/Turn/kI", 0.0);
    private final SavedTunableNumber tMaxVel = SavedTunableNumber.get("Tuning/Swerve/Turn/MaxVelocity", 0.0);
    private final SavedTunableNumber tMaxAccel = SavedTunableNumber.get("Tuning/Swerve/Turn/MaxAcceleration", 0.0);

    private final SwerveModulePosition position = new SwerveModulePosition();
    private final SwerveModuleState state = new SwerveModuleState();

    /* PID */
    private final ProfiledPIDController tProfiledPIDController =
            new ProfiledPIDController(tP.get(), tI.get(), 0.0, new Constraints(tMaxVel.get(), tMaxAccel.get()));

    private final PIDController dPIDController = new PIDController(dP.get(), dI.get(), 0.0);

    private double targetSpeedMps = 0.0;
    private Rotation2d targetAngle = new Rotation2d();

    /**
     * Create a new swerve module
     *
     * @param config The {@link SwerveModuleConfig} for the module
     */
    public SwerveModule(final SwerveModuleConfig config) {
        this.offset = config.offset();
        this.moduleNumber = config.moduleNumber();

        this.encoder = new CANcoder(config.encoderId(), canBus);
        this.driveMotor = new TalonFX(config.driveMotorId(), canBus);
        this.turningMotor = new TalonFX(config.turningMotorId(), canBus);
        this.resetToAbs();
        this.driveMotor.setPosition(0.0);
        dP.addRefreshable(this);
        dI.addRefreshable(this);
        tP.addRefreshable(this);
        tI.addRefreshable(this);
        tMaxVel.addRefreshable(this);
        tMaxAccel.addRefreshable(this);
        dFFkS.addRefreshable(this);
        dFFkV.addRefreshable(this);
        tFFkS.addRefreshable(this);
        tFFkV.addRefreshable(this);
    }

    @Override
    /** Update motor and PID configurations from NetworkTables */
    public void refresh() {
        if (this.driveMotor == null) return;

        System.out.printf("Refreshing %s\n", dP.get());

        SwerveConstants.init();
        this.driveMotor.getConfigurator().apply(SwerveConstants.DRIVE_MOTOR_CONFIG);
        this.turningMotor.getConfigurator().apply(SwerveConstants.TURNING_MOTOR_CONFIG);
        this.encoder.getConfigurator().apply(SwerveConstants.CANCODER_CONFIG);

        this.driveFF.setKs(dFFkS.get());
        this.driveFF.setKv(dFFkV.get());

        this.turnFF.setKs(tFFkS.get());
        this.turnFF.setKv(tFFkV.get());

        this.tProfiledPIDController.setConstraints(new Constraints(tMaxVel.get(), tMaxAccel.get()));
        this.tProfiledPIDController.setPID(tP.get(), tI.get(), 0.0);
        this.tProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.dPIDController.setPID(dP.get(), dI.get(), 0.0);
    }

    /**
     * Set the desired state for the module
     *
     * @param desiredState The desired state
     */
    public void setDesiredState(final SwerveModuleState desiredState) {
        desiredState.optimize(getRotation());
        this.setSpeed(desiredState.speedMetersPerSecond);
        this.setAngle(desiredState.angle);
    }

    /**
     * Set the speed of the module (drive)
     *
     * @param speed The desired speed in <strong>m/s</strong>
     */
    private void setSpeed(final double speed) {
        Logger.recordOutput("SwerveSubsystem/Module " + moduleNumber + "/TargetSpeed", speed);
        this.targetSpeedMps = speed;
    }

    /**
     * Set the angle
     *
     * @param angle The angle as a {@link Rotation2d}
     */
    public void setAngle(final Rotation2d angle) {
        Logger.recordOutput("SwerveSubsystem/Module " + moduleNumber + "/TargetAngle", angle.getDegrees());
        Logger.recordOutput(
                "SwerveSubsystem/Module " + moduleNumber + "/TargetAngleRaw", angle.getDegrees() - offset.getDegrees());

        this.targetAngle = angle;
    }

    /**
     * Get the current rotation of the module
     *
     * @return The rotation of the module
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble() + offset.getRotations());
    }

    /** Reset the module to the absoulte position */
    public void resetToAbs() {
        final double corrected = encoder.getAbsolutePosition().getValueAsDouble() + offset.getRotations();
        turningMotor.setPosition(corrected);
    }

    /**
     * Get the current state of the module
     *
     * @return The state
     */
    public SwerveModuleState getState() {
        state.speedMetersPerSecond =
                MathUtils.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), SwerveConstants.WHEEL_CIRCUMFERENCE);
        state.angle = this.getRotation();
        return state;
    }

    /**
     * Get the current position of the module
     *
     * @return The position
     */
    public SwerveModulePosition getPosition() {
        position.distanceMeters =
                MathUtils.RPSToMPS(driveMotor.getPosition().getValueAsDouble(), SwerveConstants.WHEEL_CIRCUMFERENCE);
        position.angle = this.getRotation();
        return position;
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    public void setDriveVolts(final double volts) {
        this.driveMotor.setVoltage(volts);
    }

    public double getDriveVolts() {
        return this.driveMotor.getMotorVoltage().getValueAsDouble();
    }

    public void setTurnVolts(final double volts) {
        this.turningMotor.setVoltage(volts);
    }

    public void periodic() {
        final double turnMeasurement = this.getRotation().getRadians();
        final double turnSetpoint = this.targetAngle.getRadians();

        final double turnPidOut = tProfiledPIDController.calculate(turnMeasurement, turnSetpoint);
        final double turnFFOut = this.turnFF.calculate(tProfiledPIDController.getSetpoint().velocity);
        final double turnVoltage = turnPidOut + turnFFOut;
        turningMotor.setVoltage(turnVoltage);

        final double driveMeasurement = getState().speedMetersPerSecond;

        final double drivePidOut = dPIDController.calculate(driveMeasurement, this.targetSpeedMps);
        final double driveFFOut = driveFF.calculate(this.targetSpeedMps);
        final double driveVoltage = drivePidOut + driveFFOut;
        driveMotor.setVoltage(driveVoltage);

        // Logger.recordOutput(
        //         "SwerveSubsystem/Module" + moduleNumber + "/Drive/Current",
        //         driveMotor.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput(
        //         "SwerveSubsystem/Module" + moduleNumber + "/Turn/Current",
        //         turningMotor.getSupplyCurrent().getValueAsDouble());

        // Logger.recordOutput("SwerveSubsystem/Module " + moduleNumber + "/Drive/PIDOut", drivePidOut);
        // Logger.recordOutput("SwerveSubsystem/Module " + moduleNumber + "/Drive/VoltageOut", driveVoltage);

        // Logger.recordOutput("SwerveSubsystem/Module " + moduleNumber + "/Drive/ReportedMPS", driveMeasurement);
        // Logger.recordOutput(
        //         "SwerveSubsystem/Module " + moduleNumber + "/Drive/ReportedRPS",
        //         this.driveMotor.getVelocity().getValueAsDouble());
        // Logger.recordOutput("SwerveSubsystem/Module " + moduleNumber + "/Drive/TargetMPS", this.targetSpeedMps);

        // Logger.recordOutput("SwerveSubsystem/Module " + moduleNumber + "/Drive/PIDError", dPIDController.getError());
        // Logger.recordOutput(
        //         "SwerveSubsystem/Module " + moduleNumber + "/Turn/PIDError",
        // tProfiledPIDController.getPositionError());

        // Logger.recordOutput(
        //         "SwerveSubsystem/Module " + moduleNumber + "/Turn/PIDSetpoint",
        // Units.radiansToDegrees(turnSetpoint));

        // Logger.recordOutput(
        //         "SwerveSubsystem/Module " + moduleNumber + "/Turn/MeasurementDeg",
        //         this.getRotation().getDegrees());

        // // Current monitoring
        // Logger.recordOutput(
        //         "Debug/Current/Swerve/Module " + moduleNumber + "/Drive",
        //         driveMotor.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput(
        //         "Debug/Current/Swerve/Module " + moduleNumber + "/Turn",
        //         turningMotor.getSupplyCurrent().getValueAsDouble());
    }

    /**
     * A config for a module
     *
     * @param offset The rotational offset from zero for the module
     * @param driveMotorId The CAN id of the drive motor
     * @param turningMotorId The CAN id of the turning motor
     * @param encoderId The CAN id of the encoder
     */
    public record SwerveModuleConfig(
            int moduleNumber, Rotation2d offset, int driveMotorId, int turningMotorId, int encoderId) {}
}
