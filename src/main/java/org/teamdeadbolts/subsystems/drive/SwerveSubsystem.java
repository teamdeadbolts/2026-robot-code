/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class SwerveSubsystem extends SubsystemBase {
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private SwerveModule[] modules;
    private SlewRateLimiter slewRateLimiterTranslationalX;
    private SlewRateLimiter slewRateLimiterTranslationalY;
    private SlewRateLimiter slewRateLimiterRotaional;

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

    private SysIdRoutine driveRoutine = new SysIdRoutine(
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

        trajHeadingController.setPID(trajRotP.get(), trajRotI.get(), trajRotD.get());
        trajXController.setPID(trajTransP.get(), trajTransI.get(), trajTransD.get());
        trajYController.setPID(trajTransP.get(), trajTransI.get(), trajTransD.get());

        ConfigManager.getInstance().onReady(() -> this.refreshTuning());

        slewRateRotaional.onChange(rs -> this.slewRateLimiterRotaional = new SlewRateLimiter(rs));
        slewRateTranslational.onChange(rt -> {
            this.slewRateLimiterTranslationalX = new SlewRateLimiter(rt);
            this.slewRateLimiterTranslationalY = new SlewRateLimiter(rt);
        });

        trajTransP.onChange((p) -> {
            trajXController.setP(p);
            trajYController.setP(p);
        });

        trajTransI.onChange((i) -> {
            trajXController.setI(i);
            trajYController.setI(i);
        });

        trajTransD.onChange((d) -> {
            trajXController.setD(d);
            trajYController.setD(d);
        });

        trajRotP.onChange(trajHeadingController::setP);
        trajRotI.onChange(trajHeadingController::setI);
        trajRotD.onChange(trajHeadingController::setD);
    }

    public void reconfigure() {
        trajHeadingController.setPID(trajRotP.get(), trajRotI.get(), trajRotD.get());
        trajXController.setPID(trajTransP.get(), trajTransI.get(), trajTransD.get());
        trajYController.setPID(trajTransP.get(), trajTransI.get(), trajTransD.get());

        for (SwerveModule module : this.modules) {
            module.configure();
        }
    }

    /**
     * Make the robot drive
     * @param translation A {@link Translation2d} representing the x and y motion in <strong>m/s</strong>
     * @param rotation The target rotation speed for the motion in <strong>rads/sec</strong>
     * @param fieldRelative Weather or not to drive the robot relative to the field or itself
     * @param useOdometryRotation Weather or not to use the gyro rotation or the full odometry rotation
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean slewRates, boolean useOdometryRotation) {
        // Logger.recordOutput("Swerve/CommandedVelocitiesTrans", translation);
        // Logger.recordOutput("Swerve/CommandedVelocitiesRot", Units.radiansToDegrees(rotation));

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
            speeds = fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            slewRateLimiterTranslationalX.calculate(speeds.vxMetersPerSecond),
                            slewRateLimiterTranslationalY.calculate(speeds.vyMetersPerSecond),
                            slewRateLimiterRotaional.calculate(speeds.omegaRadiansPerSecond),
                            robotRotation)
                    : new ChassisSpeeds(
                            slewRateLimiterTranslationalX.calculate(speeds.vxMetersPerSecond),
                            slewRateLimiterTranslationalY.calculate(speeds.vyMetersPerSecond),
                            slewRateLimiterRotaional.calculate(speeds.omegaRadiansPerSecond));
        } else {
            speeds = fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond,
                            robotRotation)
                    : speeds;
        }
        return speeds;
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d currPose = RobotState.getInstance().getRobotPose().toPose2d();

        double xPidOut = trajXController.calculate(currPose.getX(), sample.x);
        double yPidOut = trajYController.calculate(currPose.getY(), sample.y);
        double headingPidOut =
                trajHeadingController.calculate(currPose.getRotation().getRadians(), sample.heading);

        ChassisSpeeds speeds =
                new ChassisSpeeds(sample.vx + xPidOut, sample.vy + yPidOut, sample.omega + headingPidOut);

        this.drive(speeds, true, false, false);

        Logger.recordOutput("Swerve/TrajFollow/xPidOut", xPidOut);
        Logger.recordOutput("Swerve/TrajFollow/yPidOut", yPidOut);
        Logger.recordOutput("Swerve/TrajFollow/headingPidOut", headingPidOut);

        Logger.recordOutput("Swerve/TrajFollow/xError", currPose.getX() - sample.x);
        Logger.recordOutput("Swerve/TrajFollow/yError", currPose.getY() - sample.y);
        Logger.recordOutput(
                "Swerve/TrajFollow/headingError", currPose.getRotation().getRadians() - sample.heading);
        Pose2d samplePose = new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading));
        Logger.recordOutput("Swerve/TrajFollow/SampleTranslation", samplePose);
        Logger.recordOutput("Swerve/TrajFollow/TargetRotation", sample.omega);
        Logger.recordOutput("Swerve/TrajFollow/TargetVX", sample.vx);
        Logger.recordOutput("Swerve/TrajFollow/TargetVY", sample.vy);
    }

    /**
     * Resets the gyro
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Get the rotation of the robot
     * @return The rotation
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Get the states of all of the modules
     * @return 4 {@link SwerveModuleState}s (front left, front right, back left, back right)
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule m : this.modules) {
            states[m.getModuleNumber()] = m.getState();
        }
        return states;
    }

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

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        ChassisSpeeds robotRelative = this.getRobotRelativeChassisSpeeds();
        return new ChassisSpeeds(
                robotRelative.vxMetersPerSecond * Math.cos(getGyroRotation().getRadians())
                        - robotRelative.vyMetersPerSecond
                                * Math.sin(getGyroRotation().getRadians()),
                robotRelative.vyMetersPerSecond * Math.cos(getGyroRotation().getRadians())
                        + robotRelative.vxMetersPerSecond
                                * Math.sin(getGyroRotation().getRadians()),
                robotRelative.omegaRadiansPerSecond);
    }

    /**
     * Refresh the tuning values from AdvantageKit
     */
    public void refreshTuning() {
        // if (a) CtreConfigs.init();
        // System.out.println("Refreshing tuning");
        // this.slewRateLimiterTranslationalX = new SlewRateLimiter(slewRateTranslational.get());
        // this.slewRateLimiterTranslationalY = new SlewRateLimiter(slewRateTranslational.get());
        // this.slewRateLimiterRotaional = new SlewRateLimiter(slewRateRotaional.get());

        // for (SwerveModule m : modules) {
        //     m.configure();
        // }
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

    // Sysid functions
    private void sysIdDriveVolts(Voltage voltage) {
        for (SwerveModule m : this.modules) {
            m.setAngle(new Rotation2d());
            m.setDriveVolts(voltage.baseUnitMagnitude());
        }
    }

    private void sysIdDriveLog(SysIdRoutineLog log) {
        SwerveModule m = this.modules[0]; // Just use values from 1 module
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
