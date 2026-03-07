/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Manages the swerve drive drivetrain, including kinematics, odometry updates,
 * trajectory following, and system identification.
 */
public class SwerveSubsystem extends SubsystemBase implements Refreshable {
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final SwerveModule[] modules;
    private SlewRateLimiter slewRateLimiterTranslationalX;
    private SlewRateLimiter slewRateLimiterTranslationalY;
    private SlewRateLimiter slewRateLimiterRotaional;

    /* --- Tuning Parameters --- */
    private final SavedLoggedNetworkNumber maxModuleSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Swerve/MaxModuleSpeed", 1.0);
    private final SavedLoggedNetworkNumber slewRateTranslational =
            SavedLoggedNetworkNumber.get("Tuning/Swerve/TranslationSlew", 1.0);
    private final SavedLoggedNetworkNumber slewRateRotaional =
            SavedLoggedNetworkNumber.get("Tuning/Swerve/RotationSlew", 1.0);

    private final PIDController trajXController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController trajYController = new PIDController(0.0, 0.0, 0.0);
    private final PIDController trajHeadingController = new PIDController(0, 0.0, 0.0);

    private final SavedLoggedNetworkNumber trajTransP =
            SavedLoggedNetworkNumber.get("Tuning/Choreo/Translation/kP", 0.0);
    private final SavedLoggedNetworkNumber trajTransI = SavedLoggedNetworkNumber.get("Tuning/Choreo/Translation/kI", 0);
    private final SavedLoggedNetworkNumber trajTransD = SavedLoggedNetworkNumber.get("Tuning/Choreo/Translation/kD", 0);

    private final SavedLoggedNetworkNumber trajRotP = SavedLoggedNetworkNumber.get("Tuning/Choreo/Rotation/kP", 0);
    private final SavedLoggedNetworkNumber trajRotI = SavedLoggedNetworkNumber.get("Tuning/Choreo/Rotation/kI", 0);
    private final SavedLoggedNetworkNumber trajRotD = SavedLoggedNetworkNumber.get("Tuning/Choreo/Rotation/kD", 0);

    private final SysIdRoutine driveRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, Time.ofBaseUnits(1.5, Seconds)),
            new SysIdRoutine.Mechanism(this::sysIdDriveVolts, this::sysIdDriveLog, this));

    public SwerveSubsystem() {
        this.resetGyro();
        this.modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.FRONT_LEFT_CONFIG),
            new SwerveModule(SwerveConstants.FRONT_RIGHT_CONFIG),
            new SwerveModule(SwerveConstants.BACK_LEFT_CONFIG),
            new SwerveModule(SwerveConstants.BACK_RIGHT_CONFIG)
        };

        trajHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        slewRateTranslational.addRefreshable(this);
        slewRateRotaional.addRefreshable(this);
        //        refresh();
    }

    @Override
    public void refresh() {
        trajHeadingController.setPID(trajRotP.get(), trajRotI.get(), trajRotD.get());
        trajXController.setPID(trajTransP.get(), trajTransI.get(), trajTransD.get());
        trajYController.setPID(trajTransP.get(), trajTransI.get(), trajTransD.get());

        this.slewRateLimiterTranslationalX = new SlewRateLimiter(slewRateTranslational.get());
        this.slewRateLimiterTranslationalY = new SlewRateLimiter(slewRateTranslational.get());
        this.slewRateLimiterRotaional = new SlewRateLimiter(Units.degreesToRadians(slewRateRotaional.get()));
    }

    /**
     * Commands the drivetrain to move with specific chassis speeds.
     * * @param speeds The target {@link ChassisSpeeds}.
     * @param fieldRelative True to drive relative to the field, false for robot-relative.
     * @param slewRates True to apply slew rate limiting for smoother motion.
     * @param useOdometryRotation True to use odometry rotation, false to use raw gyro.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean slewRates, boolean useOdometryRotation) {
        SwerveModuleState[] states = SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(calculateChassisSpeeds(
                speeds,
                slewRates,
                fieldRelative,
                useOdometryRotation
                        ? RobotState.getInstance().getRobotPose().getRotation().toRotation2d()
                        : getGyroRotation()));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxModuleSpeed.get());
        for (SwerveModule m : modules) {
            m.setDesiredState(states[m.getModuleNumber()]);
        }
    }

    private ChassisSpeeds calculateChassisSpeeds(
            ChassisSpeeds speeds, boolean slewRates, boolean fieldRelative, Rotation2d robotRotation) {
        if (slewRates) {
            return fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            slewRateLimiterTranslationalX.calculate(speeds.vxMetersPerSecond),
                            slewRateLimiterTranslationalY.calculate(speeds.vyMetersPerSecond),
                            slewRateLimiterRotaional.calculate(speeds.omegaRadiansPerSecond),
                            robotRotation)
                    : new ChassisSpeeds(
                            slewRateLimiterTranslationalX.calculate(speeds.vxMetersPerSecond),
                            slewRateLimiterTranslationalY.calculate(speeds.vyMetersPerSecond),
                            slewRateLimiterRotaional.calculate(speeds.omegaRadiansPerSecond));
        }
        return fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, robotRotation)
                : speeds;
    }

    /** Resets the gyro yaw. */
    public void resetGyro() {
        gyro.reset();
    }

    /** @return The current gyro rotation. */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /** @return The array of module states. */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule m : this.modules) {
            states[m.getModuleNumber()] = m.getState();
        }
        return states;
    }

    /** @return The array of module positions. */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule m : modules) {
            positions[m.getModuleNumber()] = m.getPosition();
        }
        return positions;
    }

    public void resetModulesToAbs() {
        for (SwerveModule m : modules) {
            m.resetToAbs();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, this.maxModuleSpeed.get());
        for (SwerveModule m : this.modules) {
            m.setDesiredState(desiredStates[m.getModuleNumber()]);
        }
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return SwerveConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /** @return Chassis speeds in the field-relative frame. */
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        ChassisSpeeds robotRelative = this.getRobotRelativeChassisSpeeds();
        double rot = getGyroRotation().getRadians();
        return new ChassisSpeeds(
                robotRelative.vxMetersPerSecond * Math.cos(rot) - robotRelative.vyMetersPerSecond * Math.sin(rot),
                robotRelative.vyMetersPerSecond * Math.cos(rot) + robotRelative.vxMetersPerSecond * Math.sin(rot),
                robotRelative.omegaRadiansPerSecond);
    }

    public SwerveModule getModule(int id) {
        return this.modules[id];
    }

    public Command runDriveQuasiTest(Direction direction) {
        return driveRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(Direction direction) {
        return driveRoutine.dynamic(direction);
    }

    private void sysIdDriveVolts(Voltage voltage) {
        for (SwerveModule m : this.modules) {
            m.setAngle(new Rotation2d());
            m.setDriveVolts(voltage.baseUnitMagnitude());
        }
    }

    private void sysIdDriveLog(SysIdRoutineLog log) {
        SwerveModule m = this.modules[0];
        log.motor("Module0").linearPosition(Distance.ofBaseUnits(m.getPosition().distanceMeters, Meters));
        log.motor("Module0")
                .linearVelocity(LinearVelocity.ofBaseUnits(m.getState().speedMetersPerSecond, MetersPerSecond));
        log.motor("Module0").voltage(Voltage.ofBaseUnits(m.getDriveVolts(), Volts));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Swerve/GyroRotationDeg", getGyroRotation().getDegrees());
        for (SwerveModule m : this.modules) {
            m.periodic();
        }

        RobotState.getInstance().updateFromSwerve(getModulePositions(), new Rotation3d(getGyroRotation()));
        ChassisSpeeds speeds = getFieldRelativeChassisSpeeds();
        RobotState.getInstance().setFieldRelativeVelocities(speeds);
        RobotState.getInstance().setRobotRelativeVelocities(getRobotRelativeChassisSpeeds());
        Logger.recordOutput("Swerve/RobotVelocities", speeds);
    }
}
