/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.logstructs.shooter.ShooterAprilTagData;
import org.teamdeadbolts.subsystems.logstructs.shooter.ShooterFallbackData;
import org.teamdeadbolts.subsystems.logstructs.shooter.ShooterHoodData;
import org.teamdeadbolts.subsystems.logstructs.shooter.ShooterSummaryData;
import org.teamdeadbolts.subsystems.logstructs.shooter.ShooterTurretCalData;
import org.teamdeadbolts.subsystems.logstructs.shooter.ShooterTurretData;
import org.teamdeadbolts.subsystems.logstructs.shooter.ShooterWheelData;
import org.teamdeadbolts.utils.PeriodicTasks;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.Zone;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

/**
 * Manages the multi-DOF shooter subsystem, including turret rotation, hood
 * elevation,
 * and dual-wheel flywheel speed control. Provides automated targeting based on
 * AprilTag tracking and field zones.
 */
public class ShooterSubsystem extends StatefulSubsystem<ShooterSubsystem.State> implements Refreshable {
    public enum State {
        OFF,
        DOWN,
        APRILTAG_TRACK,
        SPINUP,
        SHOOT,
        PASS,
        ZERO,
        TEST,
        SYSTEMS_TEST
    }

    private final CANBus rio = new CANBus("rio");
    private final CANBus canivore = new CANBus("*");
    private final TalonFX turretMotor = new TalonFX(ShooterConstants.SHOOTER_TURRET_MOTOR_CAN_ID, canivore);
    private final TalonFX hoodMotor = new TalonFX(ShooterConstants.SHOOTER_HOOD_MOTOR_CAN_ID, rio);
    private final TalonFX leftWheelMotor = new TalonFX(ShooterConstants.SHOOTER_WHEEL_MOTOR_LEFT_CAN_ID, rio);
    private final TalonFX rightWheelMotor = new TalonFX(ShooterConstants.SHOOTER_WHEEL_MOTOR_RIGHT_CAN_ID, rio);

    private final StatusSignal<AngularVelocity> turretVelocitySignal = turretMotor.getVelocity();
    private final StatusSignal<Angle> turretPositionSignal = turretMotor.getPosition();
    private final StatusSignal<Current> turretCurrentSignal = turretMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> hoodVelocitySignal = hoodMotor.getVelocity();
    private final StatusSignal<Angle> hoodPositionSignal = hoodMotor.getPosition();
    private final StatusSignal<Current> hoodCurrentSignal = hoodMotor.getSupplyCurrent();

    private final StatusSignal<AngularVelocity> wheelVelocitySignal = leftWheelMotor.getVelocity();
    private final StatusSignal<Current> wheelCurrentSignal = leftWheelMotor.getSupplyCurrent();

    private final List<BaseStatusSignal> rioSignals =
            List.of(hoodVelocitySignal, hoodPositionSignal, hoodCurrentSignal, wheelVelocitySignal, wheelCurrentSignal);

    private final List<BaseStatusSignal> canivoreSignals =
            List.of(turretCurrentSignal, turretPositionSignal, turretCurrentSignal, turretVelocitySignal);

    private final ProfiledPIDController hoodController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    private final ProfiledPIDController turretController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(0, 0, 0);
    private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(0, 0, 0);

    /* --- Tuning Parameters --- */
    private final SavedTunableNumber hoodControllerP = SavedTunableNumber.get("Tuning/Shooter/HoodController/kP", 0.1);
    private final SavedTunableNumber hoodControllerI = SavedTunableNumber.get("Tuning/Shooter/HoodController/kI", 0.0);
    private final SavedTunableNumber hoodControllerD = SavedTunableNumber.get("Tuning/Shooter/HoodController/kD", 0.0);
    private final SavedTunableNumber hoodControllerMaxVel =
            SavedTunableNumber.get("Tuning/Shooter/HoodController/MaxVel", 720);
    private final SavedTunableNumber hoodControllermAccel =
            SavedTunableNumber.get("Tuning/Shooter/HoodController/MaxAccel", 360);
    private final SavedTunableNumber hoodControllerTol =
            SavedTunableNumber.get("Tuning/Shooter/HoodController/ToleranceDeg", 0.0);
    private final SavedTunableNumber hoodFeedforwardKs = SavedTunableNumber.get("Tuning/Shooter/HoodFeedforward/Ks", 0);
    private final SavedTunableNumber hoodZeroVoltage = SavedTunableNumber.get("Tuning/Shooter/HoodZeroVoltage", 0.0);
    private final SavedTunableNumber hoodZeroCurrent = SavedTunableNumber.get("Tuning/Shooter/HoodZeroCurrent", 0.0);
    private final SavedTunableNumber hoodZeroVelTol = SavedTunableNumber.get("Tuning/Shooter/HoodZeroVelTol", 0.0);

    private final SavedTunableNumber turretControllerP =
            SavedTunableNumber.get("Tuning/Shooter/TurretController/kP", 0.1);
    private final SavedTunableNumber turretControllerI =
            SavedTunableNumber.get("Tuning/Shooter/TurretController/kI", 0.0);
    private final SavedTunableNumber turretControllerD =
            SavedTunableNumber.get("Tuning/Shooter/TurretController/kD", 0.0);
    private final SavedTunableNumber turretFFS = SavedTunableNumber.get("Tuning/Shooter/TurretController/kS", 0.0);
    private final SavedTunableNumber turretFFV = SavedTunableNumber.get("Tuning/Shooter/TurretController/kV", 0.0);
    private final SavedTunableNumber turretFFA = SavedTunableNumber.get("Tuning/Shooter/TurretController/kA", 0.0);

    private final SavedTunableNumber turretMaxVel = SavedTunableNumber.get("Tuning/Shooter/TurretController/MaxVel", 0);
    private final SavedTunableNumber turretMaxAccel =
            SavedTunableNumber.get("Tuning/Shooter/TurretController/MaxAccel", 0);
    private final SavedTunableNumber turretIZone = SavedTunableNumber.get("Tuning/Shooter/TurretController/IZone", 0.0);
    private final SavedTunableNumber turretIMax =
            SavedTunableNumber.get("Tuning/Shooter/TurretController/IMaxVolts", 3);
    private final SavedTunableNumber turretIMin =
            SavedTunableNumber.get("Tuning/Shooter/TurretController/IMinVolts", 0);

    private final SavedTunableNumber wheelFFS = SavedTunableNumber.get("Tuning/Shooter/WheelController/kS", 0.1);
    private final SavedTunableNumber wheelFFV = SavedTunableNumber.get("Tuning/Shooter/WheelController/kV", 1);
    private final SavedTunableNumber wheelFFA = SavedTunableNumber.get("Tuning/Shooter/WheelController/kA", 0.0);

    private final SavedTunableNumber bangTol = SavedTunableNumber.get("Tuning/Shooter/BangBangTol", 100);
    private final SavedTunableNumber shooterWheelSpinupSpeed =
            SavedTunableNumber.get("Tuning/Shooter/ShooterWheelSpinupSpeed", 5000.0);

    private final SavedTunableNumber testHoodAngle = SavedTunableNumber.get("Tuning/Shooter/TestHoodAngle", 45);
    private final SavedTunableNumber testShooterMPS = SavedTunableNumber.get("Tuning/Shooter/TestShooterRPM", 3);
    private final SavedTunableNumber testTurretAngle = SavedTunableNumber.get("Tuning/Shooter/TestTurretAngle", 0);

    private final SavedTunableNumber aprilTagTrackRange =
            SavedTunableNumber.get("Tuning/Shooter/AprilTagTrackRange", 0);

    private final SavedTunableNumber alternativeMinHoodAngle =
            SavedTunableNumber.get("Tuning/Shooter/AlternativeMinHoodAngleDeg", 23);

    private Optional<Double> targetWheelSpeed;
    private Optional<Double> targetTurretPosition;
    private double currentWheelSpeed;
    private double currentTurretPosition;
    private final Zone aprilTagTrackZone = new Zone();
    private final ShotCalculator shotCalculator;
    private int systemTestCount = 0;

    private boolean turretFallbackMode = false;
    private boolean alternative = false;
    private boolean isPossibleShot = false;
    private Optional<Rotation2d> fallbackChassisTargetAngle = Optional.empty();

    public ShooterSubsystem() {
        super(State.DOWN);
        this.shotCalculator = new ShotCalculator();

        hoodMotor.setPosition(Units.degreesToRotations(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
        resetTurretPosition();

        hoodControllerP.addRefreshable(this);
        hoodControllerI.addRefreshable(this);
        hoodControllerD.addRefreshable(this);
        hoodControllerTol.addRefreshable(this);
        hoodControllerMaxVel.addRefreshable(this);
        hoodControllermAccel.addRefreshable(this);
        hoodFeedforwardKs.addRefreshable(this);
        turretControllerP.addRefreshable(this);
        turretControllerI.addRefreshable(this);
        turretControllerD.addRefreshable(this);
        turretFFS.addRefreshable(this);
        turretFFV.addRefreshable(this);
        turretFFA.addRefreshable(this);
        turretIMin.addRefreshable(this);
        turretIMax.addRefreshable(this);
        turretIZone.addRefreshable(this);
        turretMaxVel.addRefreshable(this);
        turretMaxAccel.addRefreshable(this);
        wheelFFS.addRefreshable(this);
        wheelFFV.addRefreshable(this);
        wheelFFA.addRefreshable(this);
    }

    @Override
    public void refresh() {
        hoodController.setPID(hoodControllerP.get(), hoodControllerI.get(), hoodControllerD.get());
        hoodController.setTolerance(Units.degreesToRadians(hoodControllerTol.get()));
        hoodController.setConstraints(new TrapezoidProfile.Constraints(
                Units.degreesToRadians(hoodControllerMaxVel.get()),
                Units.degreesToRadians(hoodControllermAccel.get())));

        turretController.setPID(turretControllerP.get(), turretControllerI.get(), turretControllerD.get());
        turretController.setConstraints(new TrapezoidProfile.Constraints(
                Units.degreesToRadians(turretMaxVel.get()), Units.degreesToRadians(turretMaxAccel.get())));
        turretController.setIZone(turretIZone.get());
        turretController.setIntegratorRange(turretIMin.get(), turretIMax.get());
        turretController.setTolerance(0);
        turretFF.setKs(turretFFS.get());
        turretFF.setKv(turretFFV.get());
        turretFF.setKa(turretFFA.get());

        wheelFF.setKs(wheelFFS.get());
        wheelFF.setKv(wheelFFV.get());
        wheelFF.setKa(wheelFFA.get());

        ShooterConstants.init();
        hoodMotor.getConfigurator().apply(ShooterConstants.SHOOTER_HOOD_MOTOR_CONFIG);
        turretMotor.getConfigurator().apply(ShooterConstants.SHOOTER_TURRET_MOTOR_CONFIG);
        leftWheelMotor.getConfigurator().apply(ShooterConstants.SHOOTER_WHEEL_MOTOR_CONFIG);
        rightWheelMotor.getConfigurator().apply(ShooterConstants.SHOOTER_WHEEL_MOTOR_CONFIG);
        rightWheelMotor.setControl(
                new Follower(ShooterConstants.SHOOTER_WHEEL_MOTOR_LEFT_CAN_ID, MotorAlignmentValue.Opposed));
    }

    public void rezeroTurret() {
        //     HashMap<String, PhotonVisionIOCtxAutoLogged> ios =
        //             RobotState.getInstance().getIos();
        //     PhotonVisionIOCtxAutoLogged backCamIo = ios.get("Back Cam");
    }

    public double getRPMError() {
        if (targetWheelSpeed.isEmpty()) return 0;
        return (targetWheelSpeed.get() - Units.radiansPerSecondToRotationsPerMinute(currentWheelSpeed));
    }

    public double getTurretError() {
        if (targetTurretPosition.isEmpty()) return 0;
        final double normalizedSetpoint = calculateTurretSetpoint(currentTurretPosition, targetTurretPosition.get());
        return Math.abs(normalizedSetpoint - currentTurretPosition);
    }

    public boolean isTurretSettled(double positionToleranceRad, double velocityToleranceRadPerSec) {
        if (targetTurretPosition.isEmpty()) return false;

        double positionError = getTurretError();
        double currentVelocity = Math.abs(Units.rotationsToRadians(turretVelocitySignal.getValueAsDouble()));

        Logger.recordOutput("ShooterSubsystem/Settled/Pos", positionError <= positionToleranceRad);
        Logger.recordOutput("ShooterSubsystem/Settled/Vel", currentVelocity <= velocityToleranceRadPerSec);
        Logger.recordOutput("ShooterSubsystem/Settled/PosError", Units.radiansToDegrees(positionError));

        return positionError <= positionToleranceRad && currentVelocity <= velocityToleranceRadPerSec;
    }

    public boolean isPossibleShot() {
        return isPossibleShot;
    }

    public void resetTurretPosition() {
        this.turretMotor.setPosition(0);
    }

    public void setTurretFallbackMode(final boolean enabled) {
        this.turretFallbackMode = enabled;
    }

    public void setAlternativeHoodMinAngle(final boolean alternative) {
        this.alternative = alternative;
    }

    public boolean isTurretFallbackMode() {
        return this.turretFallbackMode;
    }

    public Optional<Rotation2d> getFallbackChassisTargetAngle() {
        return this.fallbackChassisTargetAngle;
    }

    public Pose3d getFieldRelativeTurretPose() {
        final Pose3d robotPose = RobotState.getInstance().getRobotPose();

        final double fieldRelativeHeading = robotPose.getRotation().getZ() + getTurretRotation();

        return new Pose3d(
                robotPose.transformBy(ShooterConstants.SHOOTER_OFFSET).getTranslation(),
                new Rotation3d(0, 0, fieldRelativeHeading));
    }

    public Transform3d getRobotRelativeTurretTransform() {
        return new Transform3d(
                ShooterConstants.SHOOTER_OFFSET.getTranslation(), new Rotation3d(0, 0, getTurretRotation()));
    }

    @Override
    protected void onStateChange(final State from, final State to) {
        final double hoodPosition = Units.rotationsToRadians(hoodPositionSignal.getValueAsDouble());
        final double hoodVelocity = Units.rotationsToRadians(hoodVelocitySignal.getValueAsDouble());

        hoodController.reset(hoodPosition, hoodVelocity);

        final double turretPosition = Units.rotationsToRadians(turretPositionSignal.getValueAsDouble());
        final double turretVelocity = Units.rotationsToRadians(turretVelocitySignal.getValueAsDouble());
        turretController.reset(new TrapezoidProfile.State(turretPosition, turretVelocity));
    }

    public List<BaseStatusSignal> getSignals() {
        List<BaseStatusSignal> signals = new ArrayList<>(rioSignals.size() + canivoreSignals.size());
        signals.addAll(rioSignals);
        signals.addAll(canivoreSignals);
        return signals;
    }

    @Override
    public void subsystemPeriodic() {
        PeriodicTasks periodicTasks = PeriodicTasks.getInstance();
        if (periodicTasks.shouldRefreshSignals()) {
            BaseStatusSignal.refreshAll(rioSignals);
            BaseStatusSignal.refreshAll(canivoreSignals);
        }

        final double currentHoodAngle = Units.rotationsToRadians(hoodPositionSignal.getValueAsDouble());
        Optional<Double> targetHoodAngle = Optional.empty();
        Optional<Translation3d> currentTargetTranslation = Optional.empty();

        currentWheelSpeed = Units.rotationsToRadians(wheelVelocitySignal.getValueAsDouble());
        targetWheelSpeed = Optional.empty();
        targetTurretPosition = Optional.empty();
        Optional<Double> targetTurretVelocity = Optional.empty();

        final Pose3d robotPose = RobotState.getInstance().getRobotPose();
        final ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeRobotVelocities();
        shotCalculator.updateVelocityState((double) System.currentTimeMillis(), robotSpeeds);

        final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        switch (targetState) {
            case OFF -> {
                // Do nothing
            }
            case DOWN -> {
                systemTestCount = 0;
                targetHoodAngle = Optional.of(Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
            }
            case APRILTAG_TRACK -> {
                final Pose3d turretPose = getFieldRelativeTurretPose();
                final Pose2d robotPose2d = robotPose.toPose2d();
                final Translation2d turretOffset =
                        ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
                final Rotation2d towardsCenterAngle = turretOffset.times(-1.0).getAngle();
                final Translation2d shiftRobotRel = new Translation2d(0.2245, towardsCenterAngle);
                final Translation2d vertexRobotRel = turretOffset.plus(shiftRobotRel);

                final double dirX = Math.signum(turretOffset.getX());
                final double dirY = Math.signum(turretOffset.getY());

                final Translation2d forwardLeg =
                        vertexRobotRel.plus(new Translation2d(aprilTagTrackRange.get() * dirX, 0));
                final Translation2d sideLeg =
                        vertexRobotRel.plus(new Translation2d(0, aprilTagTrackRange.get() * dirY));

                final Translation2d vertexFieldRel =
                        vertexRobotRel.rotateBy(robotPose2d.getRotation()).plus(robotPose2d.getTranslation());
                final Translation2d forwardLegFieldRel =
                        forwardLeg.rotateBy(robotPose2d.getRotation()).plus(robotPose2d.getTranslation());
                final Translation2d sideLegFieldRel =
                        sideLeg.rotateBy(robotPose2d.getRotation()).plus(robotPose2d.getTranslation());

                aprilTagTrackZone.setVertices(vertexFieldRel, forwardLegFieldRel, sideLegFieldRel);

                //                if (periodicTasks.shouldLog()) {
                //                    Logger.recordOutput("ShooterSubsystem/AprilTagTrack/RangeVertex",
                // vertexFieldRel);
                //                    Logger.recordOutput("ShooterSubsystem/AprilTagTrack/RangeForward",
                // forwardLegFieldRel);
                //                    Logger.recordOutput("ShooterSubsystem/AprilTagTrack/RangeSide",
                // sideLegFieldRel);
                //                }

                Pose3d targetPose = null;
                double minDistance = Double.MAX_VALUE;
                final Translation3d turretTrans = turretPose.getTranslation();

                for (final edu.wpi.first.apriltag.AprilTag tag : VisionConstants.FIELD_LAYOUT.getTags()) {
                    final Translation2d tagTrans2d = tag.pose.getTranslation().toTranslation2d();
                    if (aprilTagTrackZone.contains(tagTrans2d)) {
                        final double dist = tag.pose.getTranslation().getDistance(turretTrans);
                        if (dist < minDistance) {
                            minDistance = dist;
                            targetPose = tag.pose;
                        }
                    }
                }

                targetHoodAngle = Optional.of(Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                if (targetPose != null) {
                    targetTurretPosition = Optional.of(shotCalculator.calculateLatancyOffsetTurretAngle(
                            robotPose2d, targetPose.toPose2d().getTranslation(), System.currentTimeMillis()));
                    //                    if (periodicTasks.shouldLog()) {
                    //                        Logger.recordOutput("ShooterSubsystem/AprilTagTrack/TargetTagPose",
                    // targetPose);
                    //                        shooterData.aprilTagTrackTargetTagPose = targetPose;
                    //                    }
                }

                if (periodicTasks.shouldLog()) {
                    ShooterAprilTagData tagData = new ShooterAprilTagData(
                            vertexFieldRel,
                            forwardLegFieldRel,
                            sideLegFieldRel,
                            targetPose != null ? targetPose : Pose3d.kZero);
                    Logger.recordOutput("ShooterSubsystem/AprilTagTrack", (WPISerializable) tagData);
                }
            }
            case PASS -> {
                Translation3d passTargetPose;
                if (ZoneConstants.TOP_PASS_ZONE.contains(robotPose.toPose2d().getTranslation())) {
                    passTargetPose = alliance == Alliance.Red
                            ? ShooterConstants.PASS_TOP_POSE_RED
                            : ShooterConstants.PASS_TOP_POSE_BLUE;
                } else if (ZoneConstants.BOTTOM_PASS_ZONE.contains(
                        robotPose.toPose2d().getTranslation())) {
                    passTargetPose = alliance == Alliance.Red
                            ? ShooterConstants.PASS_BOTTOM_POSE_RED
                            : ShooterConstants.PASS_BOTTOM_POSE_BLUE;
                } else {
                    targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                    break;
                }

                if (periodicTasks.shouldLog()) {
                    Logger.recordOutput(
                            "ShooterSubsystem/PassTargetPose",
                            new Pose2d(passTargetPose.toTranslation2d(), new Rotation2d()));
                }

                currentTargetTranslation = Optional.of(passTargetPose);

                final ShotParametersAutoLogged passShot = shotCalculator.calculateShot(
                        robotPose,
                        passTargetPose,
                        System.currentTimeMillis(),
                        1.5,
                        Units.degreesToRadians(
                                alternative
                                        ? alternativeMinHoodAngle.get()
                                        : ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                targetHoodAngle = Optional.of(passShot.hoodAngle);
                targetTurretPosition = Optional.of(passShot.turretAngle);
                targetTurretVelocity = Optional.of(passShot.turretVelocity);
                targetWheelSpeed = Optional.of(passShot.wheelSpeed);
                isPossibleShot = passShot.isPossible;
            }
            case SHOOT -> {
                final Translation3d target =
                        alliance == Alliance.Blue ? ShooterConstants.SHOOT_POSE_BLUE : ShooterConstants.SHOOT_POSE_RED;

                final ShotParametersAutoLogged shootShot = shotCalculator.calculateShot(
                        robotPose,
                        target,
                        System.currentTimeMillis(),
                        0.05,
                        Units.degreesToRadians(
                                alternative
                                        ? alternativeMinHoodAngle.get()
                                        : ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                targetWheelSpeed = Optional.of(shootShot.wheelSpeed);
                targetHoodAngle = Optional.of(shootShot.hoodAngle);
                targetTurretPosition = Optional.of(shootShot.turretAngle);
                targetTurretVelocity = Optional.of(shootShot.turretVelocity);
                isPossibleShot = shootShot.isPossible;
            }
            case SPINUP -> {
                targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                targetHoodAngle = Optional.of(Units.degreesToRadians(testHoodAngle.get()));
            }
            case TEST -> { // Slowly rotatte turret in a circle
                targetWheelSpeed = Optional.of(testShooterMPS.get());
                targetHoodAngle = Optional.of(Units.degreesToRadians(testHoodAngle.get()));
                targetTurretPosition = Optional.of(Units.degreesToRadians(testTurretAngle.get()));
            }
            case ZERO -> {
                targetHoodAngle = Optional.empty();
                hoodMotor.setVoltage(-hoodZeroVoltage.get());
                if (Math.abs(hoodCurrentSignal.getValueAsDouble()) >= hoodZeroCurrent.get()
                        && Math.abs(hoodVelocitySignal.getValueAsDouble())
                                <= Units.degreesToRotations(hoodZeroVelTol.get())) {
                    this.setState(State.DOWN, Priority.HIGH);
                    hoodMotor.setPosition(Units.degreesToRotations(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                }
            }
            case SYSTEMS_TEST -> {
                systemTestCount++;
                targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                targetTurretPosition =
                        Optional.of((((Math.sin(((double) systemTestCount / 850) + 1)) / 2) * 520) - 260);
                targetHoodAngle = Optional.of((((Math.sin(((double) systemTestCount / 150) + 1)) / 2) * 35) + 10);
            }
        }

        fallbackChassisTargetAngle = Optional.empty();
        if (turretFallbackMode && targetTurretPosition.isPresent() && currentTargetTranslation.isPresent()) {
            final double lockedTurrentRad = Units.rotationsToRadians(turretPositionSignal.getValueAsDouble());
            fallbackChassisTargetAngle = Optional.of(calculateChassisAngleForLockedTurret(
                    robotPose.toPose2d(), currentTargetTranslation.get().toTranslation2d(), lockedTurrentRad));

            targetTurretPosition = Optional.empty();

            if (periodicTasks.shouldLog()) {
                //                Logger.recordOutput("ShooterSubsystem/Fallback/Fallback", true);
                //                Logger.recordOutput("ShooterSybsystem/Fallback/ChassisTarget",
                // fallbackChassisTargetAngle.get());
                ShooterFallbackData fallbackData = new ShooterFallbackData(true, fallbackChassisTargetAngle.get());
                Logger.recordOutput("ShooterSubsystem/Fallback", (WPISerializable) fallbackData);
            }
        }

        // --- Hardware Control ---
        if (targetHoodAngle.isPresent()) {
            final double targetHoodAngleClamped = MathUtil.clamp(
                    targetHoodAngle.get(),
                    Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES),
                    Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES));
            final double pidOutput = hoodController.calculate(currentHoodAngle, targetHoodAngleClamped);
            if (!hoodController.atSetpoint()) {
                hoodMotor.setVoltage(pidOutput);
            }

            if (periodicTasks.shouldLog()) {
                //                Logger.recordOutput(
                //                        "ShooterSubsystem/Hood/TargetHoodAngle",
                // Units.radiansToDegrees(targetHoodAngle.get()));
                //                Logger.recordOutput("ShooterSubsystem/Hood/HoodOutput", pidOutput);

                ShooterHoodData hoodData =
                        new ShooterHoodData(Units.radiansToDegrees(targetHoodAngle.get()), pidOutput);
                Logger.recordOutput("ShooterSubsystem/Hood", (WPISerializable) hoodData);
            }

        } else if (targetState != State.ZERO) {
            hoodMotor.setVoltage(0);
        }

        if (targetWheelSpeed.isPresent()) {
            final double wheelOutput =
                    (getRPMError() > bangTol.get()) ? 12.0 : wheelFF.calculate(targetWheelSpeed.get());
            leftWheelMotor.setControl(new VoltageOut(wheelOutput));

            if (periodicTasks.shouldLog()) {
                //                Logger.recordOutput("ShooterSubsystem/Wheel/Volts", wheelOutput);
                //                Logger.recordOutput("ShooterSubsystem/Wheel/TargetSpeedRPM", targetWheelSpeed.get());
                //                Logger.recordOutput("ShooterSubsystem/Wheel/RPMError", getRPMError());

                ShooterWheelData wheelData = new ShooterWheelData(wheelOutput, targetWheelSpeed.get(), getRPMError());
                Logger.recordOutput("ShooterSubsystem/Wheel", (WPISerializable) wheelData);
            }
        } else {
            leftWheelMotor.setVoltage(0);
        }

        if (targetTurretPosition.isPresent()) {
            currentTurretPosition =
                    Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble());

            // Just pass the robot-relative target directly into the new method
            final double normalizedTurretPosition =
                    calculateTurretSetpoint(currentTurretPosition, targetTurretPosition.get());

            // double turretPidOutput = turretController.calculate(currentTurretPosition, normalizedTurretPosition);
            double turretPidOutput = turretController.calculate(
                    currentTurretPosition,
                    new TrapezoidProfile.State(normalizedTurretPosition, targetTurretVelocity.orElse(0.0)));
            double velSetpoint = turretController.getSetpoint().velocity;
            double turretFFOutput = turretFF.calculate(velSetpoint);
            double combinedOutput = turretPidOutput + turretFFOutput;

            // Calculate field pose for logging
            final Transform2d targetTurretTransform = new Transform2d(
                    ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(),
                    Rotation2d.fromRadians(normalizedTurretPosition
                            + ShooterConstants.SHOOTER_OFFSET.getRotation().getZ()));
            final Pose2d targetTurretFieldPose = robotPose.toPose2d().transformBy(targetTurretTransform);

            turretMotor.setVoltage(combinedOutput);

            if (periodicTasks.shouldLog()) {
                final TrapezoidProfile.State state = turretController.getSetpoint();

                ShooterTurretData turretData = new ShooterTurretData(
                        Units.radiansToDegrees(normalizedTurretPosition),
                        targetTurretFieldPose,
                        turretPidOutput,
                        turretController.getPositionError(),
                        Units.radiansToDegrees(state.position),
                        Units.radiansToDegrees(state.velocity));
                Logger.recordOutput("ShooterSubsystem/Turret", (WPISerializable) turretData);
            }

        } else {
            turretMotor.setVoltage(0);
        }

        if (periodicTasks.shouldLog()) {
            ShooterSummaryData summaryData = new ShooterSummaryData(
                    Units.radiansToDegrees(currentHoodAngle),
                    hoodCurrentSignal.getValueAsDouble(),
                    alternative,
                    getFieldRelativeTurretPose(),
                    Units.radiansToDegrees(currentTurretPosition),
                    Units.rotationsToDegrees(turretVelocitySignal.getValueAsDouble()),
                    Units.radiansPerSecondToRotationsPerMinute(currentWheelSpeed),
                    leftWheelMotor.getMotorVoltage().getValueAsDouble(),
                    rightWheelMotor.getMotorVoltage().getValueAsDouble());

            Logger.recordOutput("ShooterSubsystem/Summary", (WPISerializable) summaryData);
        }
    }

    /**
     * Calculates the continuous encoder setpoint for the turret, choosing the
     * shortest path
     * to the target while respecting physical cable chain limits.
     * * @param currentEncoderRad The raw, un-wrapped current position of the turret
     * encoder (radians).
     *
     * @param targetRobotRelativeRad The desired angle relative to the front of the
     *                               robot (radians).
     * @return The optimal continuous setpoint for the PID controller (radians).
     */
    private double calculateTurretSetpoint(final double currentEncoderRad, final double targetRobotRelativeRad) {
        // Shift the target from the Robot Frame to the Encoder Frame
        final double offsetRad = ShooterConstants.SHOOTER_OFFSET.getRotation().getZ();
        final double targetEncoderRad = targetRobotRelativeRad - offsetRad;

        final double error = MathUtil.angleModulus(targetEncoderRad - currentEncoderRad);

        double setpoint = currentEncoderRad + error;

        final double maxLimitRad = Math.toRadians(ShooterConstants.TURRET_MAX_POSITION_DEGREES);
        final double minLimitRad = Math.toRadians(ShooterConstants.TURRET_MIN_POSITION_DEGREES);

        String limitHit = "NONE";

        if (setpoint > maxLimitRad) {
            if (setpoint - (2 * Math.PI) >= minLimitRad) {
                setpoint -= (2 * Math.PI);
            } else {
                setpoint = maxLimitRad;
                limitHit = "MAX";
                //                    Logger.recordOutput("ShooterSubsystem/Turret/LimitHit", "MAX");
            }
        } else if (setpoint < minLimitRad) {
            if (setpoint + (2 * Math.PI) <= maxLimitRad) {
                setpoint += (2 * Math.PI);
            } else {
                setpoint = minLimitRad;
                limitHit = "MIN";
                //                    Logger.recordOutput("ShooterSubsystem/Turret/LimitHit", "MIN");
            }
        }

        if (PeriodicTasks.getInstance().shouldLog()) {
            //            Logger.recordOutput(
            //                    "ShooterSubsystem/Turret/RawTargetAngle",
            // Units.radiansToDegrees(targetRobotRelativeRad));
            //            Logger.recordOutput("ShooterSubsystem/Turret/TargetEncoderDeg",
            // Units.radiansToDegrees(targetEncoderRad));
            //            Logger.recordOutput("ShooterSubsystem/Turret/ShortestPathError",
            // Units.radiansToDegrees(error));
            ShooterTurretCalData turretCalData = new ShooterTurretCalData(
                    Units.radiansToDegrees(targetRobotRelativeRad),
                    Units.radiansToDegrees(targetEncoderRad),
                    Units.radiansToDegrees(error),
                    Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble()));
            Logger.recordOutput("ShooterSubsystem/Turret/CalData", (WPISerializable) turretCalData);
        }

        return setpoint;
    }

    private double getTurretRotation() {
        return Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble())
                + ShooterConstants.SHOOTER_OFFSET.getRotation().getZ();
    }

    private Rotation2d calculateChassisAngleForLockedTurret(
            final Pose2d robotPose, final Translation2d targetLocation, final double lockedTurretAngle) {
        final Translation2d robotToTarget = targetLocation.minus(robotPose.getTranslation());
        final double distToTarget = robotToTarget.getNorm();
        final double angleToTarget = Math.atan2(robotToTarget.getY(), robotToTarget.getX());

        final Translation2d turretOffset =
                ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
        final double perpendicularOffset =
                turretOffset.getX() * Math.sin(lockedTurretAngle) - turretOffset.getY() * Math.cos(lockedTurretAngle);

        if (Math.abs(perpendicularOffset) > distToTarget) {
            return new Rotation2d(angleToTarget - lockedTurretAngle);
        }

        final double offsetCompensationAngle = Math.asin(perpendicularOffset / distToTarget);
        return new Rotation2d(angleToTarget - lockedTurretAngle + offsetCompensationAngle);
    }
}
