/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.Zone;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShooterSubsystem extends StatefulSubsystem<ShooterSubsystem.State> implements Refreshable {
    public enum State {
        OFF,
        APRILTAG_TRACK,
        SPINUP,
        SHOOT,
        PASS,
        ZERO,
        TEST,
        SYSTEMS_TEST;
    }

    private final CANBus rio = new CANBus("rio");
    private final CANBus canivore = new CANBus("*");
    private TalonFX turretMotor = new TalonFX(ShooterConstants.SHOOTER_TURRET_MOTOR_CAN_ID, canivore);
    private TalonFX hoodMotor = new TalonFX(ShooterConstants.SHOOTER_HOOD_MOTOR_CAN_ID, rio);
    private TalonFX leftWheelMotor = new TalonFX(ShooterConstants.SHOOTER_WHEEL_MOTOR_LEFT_CAN_ID, rio);
    private TalonFX rightWheelMotor = new TalonFX(ShooterConstants.SHOOTER_WHEEL_MOTOR_RIGHT_CAN_ID, rio);

    private PIDController hoodController = new PIDController(0.0, 0.0, 0.0);
    private PIDController turretController = new PIDController(0.0, 0.0, 0.0);
    private SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(0, 0, 0);

    private final SavedLoggedNetworkNumber hoodControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kP", 0.1);
    private final SavedLoggedNetworkNumber hoodControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kI", 0.0);
    private final SavedLoggedNetworkNumber hoodControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kD", 0.0);
    private final SavedLoggedNetworkNumber hoodControllerTol =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/ToleranceDeg", 0.0);

    private final SavedLoggedNetworkNumber hoodFeedforwardKs =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodFeedforward/Ks", 0);

    private final SavedLoggedNetworkNumber hoodZeroVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodZeroVoltage", 0.0);
    private final SavedLoggedNetworkNumber hoodZeroCurrent =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodZeroCurrent", 0.0);
    private final SavedLoggedNetworkNumber hoodZeroVelTol =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodZeroVelTol", 0.0);

    private final SavedLoggedNetworkNumber turretControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kP", 0.1);
    private final SavedLoggedNetworkNumber turretControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kI", 0.0);
    private final SavedLoggedNetworkNumber turretControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kD", 0.0);

    private final SavedLoggedNetworkNumber wheelFFS =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kS", 0.1);
    private final SavedLoggedNetworkNumber wheelFFV =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kV", 1);
    private final SavedLoggedNetworkNumber wheelFFA =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kA", 0.0);

    private final SavedLoggedNetworkNumber bangTol = SavedLoggedNetworkNumber.get("Tuning/Shooter/BangBangTol", 100);

    private final SavedLoggedNetworkNumber shooterWheelSpinupSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterWheelSpinupSpeed", 5000.0); // RPM

    private final SavedLoggedNetworkNumber testHoodAngle =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TestHoodAngle", 45);
    private final SavedLoggedNetworkNumber testShooterMPS =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TestShooterRPM", 3);

    private final SavedLoggedNetworkNumber testTargetX = SavedLoggedNetworkNumber.get("Tuning/Shooter/TestTargetX", 0);
    private final SavedLoggedNetworkNumber testTargetY = SavedLoggedNetworkNumber.get("Tuning/Shooter/TestTargetY", 0);

    private final SavedLoggedNetworkNumber aprilTagTrackRange =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/AprilTagTrackRange", 0);

    private Optional<Double> targetWheelSpeed;
    private double currentWheelSpeed;

    private Zone aprilTagTrackZone = new Zone();

    private ShotCalculator shotCalculator;

    private int systemTestCount = 0;

    public ShooterSubsystem() {
        this.shotCalculator = new ShotCalculator();
        this.targetState = State.OFF;

        //        ConfigManager.getInstance().onReady(this::refresh);
        //        refresh();
        resetTurretPosition();
        hoodMotor.setPosition(Units.degreesToRotations(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES - 1));
        hoodControllerP.addRefreshable(this);
        hoodControllerI.addRefreshable(this);
        hoodControllerD.addRefreshable(this);
        hoodControllerTol.addRefreshable(this);
        hoodFeedforwardKs.addRefreshable(this);
        turretControllerP.addRefreshable(this);
        turretControllerI.addRefreshable(this);
        turretControllerD.addRefreshable(this);
        wheelFFS.addRefreshable(this);
        wheelFFV.addRefreshable(this);
        wheelFFA.addRefreshable(this);
    }

    @Override
    public void refresh() {
        System.out.println("ShooterSubsystem refresh");
        hoodController.setPID(hoodControllerP.get(), hoodControllerI.get(), hoodControllerD.get());
        hoodController.setTolerance(Units.degreesToRadians(hoodControllerTol.get()));
        turretController.setPID(turretControllerP.get(), turretControllerI.get(), turretControllerD.get());
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

    public double getRPMError() {
        if (targetWheelSpeed.isEmpty()) return 0;
        return (targetWheelSpeed.get() - Units.radiansPerSecondToRotationsPerMinute(currentWheelSpeed));
    }

    public void resetTurretPosition() {
        this.turretMotor.setPosition(0);
    }

    public Pose3d getFieldRelativeTurretPose() {
        Pose3d robotPose = RobotState.getInstance().getRobotPose();
        return robotPose.transformBy(getTurretOffset());
    }

    public Transform3d getTurretOffset() {
        return new Transform3d(
                ShooterConstants.SHOOTER_OFFSET.getTranslation(), new Rotation3d(0, 0, getTurretRotation()));
    }

    @Override
    protected void onStateChange(State from, State to) {
        hoodController.reset();
        turretController.reset();
    }

    @Override
    public void periodic() {
        double currentHoodAngle =
                Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble());
        Optional<Double> targetHoodAngle = Optional.empty();

        double currentTurretPosition = getTurretRotation();
        Optional<Double> targetTurretPosition = Optional.empty();

        currentWheelSpeed =
                Units.rotationsToRadians(leftWheelMotor.getVelocity().getValueAsDouble());
        targetWheelSpeed = Optional.empty();

        Pose3d robotPose = RobotState.getInstance().getRobotPose();
        ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeRobotVelocities();
        shotCalculator.updateVelocityState((double) System.currentTimeMillis(), robotSpeeds);

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        switch (targetState) {
            case OFF:
                systemTestCount = 0;
                targetHoodAngle = Optional.of(Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                // targetWheelSpeed = Optional.
                break;
            case APRILTAG_TRACK:
                Pose3d turretPose = getFieldRelativeTurretPose();
                Pose2d robotPose2d = robotPose.toPose2d();
                Translation2d turretOffset =
                        ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d();
                Rotation2d towardsCenterAngle = turretOffset.times(-1.0).getAngle();
                Translation2d shiftRobotRel = new Translation2d(Units.inchesToMeters(3.0), towardsCenterAngle); // Temp
                Translation2d vertexRobotRel = turretOffset.plus(shiftRobotRel);

                double dirX = Math.signum(turretOffset.getX());
                double dirY = Math.signum(turretOffset.getY());

                Translation2d forwardLeg = vertexRobotRel.plus(new Translation2d(aprilTagTrackRange.get() * dirX, 0));
                Translation2d sideLeg = vertexRobotRel.plus(new Translation2d(0, aprilTagTrackRange.get() * dirY));

                Translation2d vertexFieldRel =
                        vertexRobotRel.rotateBy(robotPose2d.getRotation()).plus(robotPose2d.getTranslation());
                Translation2d forwardLegFieldRel =
                        forwardLeg.rotateBy(robotPose2d.getRotation()).plus(robotPose2d.getTranslation());
                Translation2d sideLegFieldRel =
                        sideLeg.rotateBy(robotPose2d.getRotation()).plus(robotPose2d.getTranslation());

                aprilTagTrackZone.setVertices(vertexFieldRel, forwardLegFieldRel, sideLegFieldRel);

                Logger.recordOutput("ShooterSubsystem/AprilTagTrack/RangeVertex", vertexFieldRel);
                Logger.recordOutput("ShooterSubsystem/AprilTagTrack/RangeForward", forwardLegFieldRel);
                Logger.recordOutput("ShooterSubsystem/AprilTagTrack/RangeSide", sideLegFieldRel);

                List<Pose3d> filteredTagPoses = VisionConstants.FIELD_LAYOUT.getTags().stream()
                        .map(t -> t.pose)
                        .filter(p -> aprilTagTrackZone.contains(p.toPose2d().getTranslation()))
                        .collect(Collectors.toList());

                // Find the tag pose closet to the turret pose
                Optional<Pose3d> targetPose = filteredTagPoses.stream()
                        .min(Comparator.comparingDouble(
                                p -> p.getTranslation().getDistance(turretPose.getTranslation())));

                if (targetPose.isPresent()) {
                    targetTurretPosition = Optional.of(shotCalculator.calculateLatancyOffsetTurrentAngle(
                            robotPose2d, targetPose.get().toPose2d().getTranslation(), System.currentTimeMillis()));

                    Logger.recordOutput("ShooterSubsystem/AprilTagTrack/TargetTagPose", targetPose.get());
                }

                break;
            case PASS:
                Pose3d passTargetPose = null;

                if (alliance == Alliance.Red) {
                    if (robotPose.getY() <= Units.inchesToMeters(135.594)) {
                        passTargetPose = ShooterConstants.PASS_BOTTOM_POSE_RED;
                    } else if (robotPose.getY() >= Units.inchesToMeters(182.594)) {
                        passTargetPose = ShooterConstants.PASS_TOP_POSE_RED;
                    } else {
                        targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                        break;
                    }

                } else {
                    if (robotPose.getY() <= Units.inchesToMeters(135.594)) {
                        passTargetPose = ShooterConstants.PASS_BOTTOM_POSE_BLUE;
                    } else if (robotPose.getY() >= Units.inchesToMeters(182.594)) {
                        passTargetPose = ShooterConstants.PASS_TOP_POSE_BLUE;
                    } else {
                        targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                        break;
                    }
                }

                ShotParametersAutoLogged passShot = shotCalculator.calculateShot(
                        robotPose, passTargetPose.getTranslation(), System.currentTimeMillis(), 0.0);
                targetHoodAngle = Optional.of(passShot.hoodAngle);
                targetTurretPosition = Optional.of(passShot.turretAngle);
                targetWheelSpeed = Optional.of(passShot.wheelSpeed);
                Logger.processInputs("ShooterSubsystem/Shot", passShot);
                break;
            case SHOOT:
                Pose3d shootTargetPose =
                        (alliance == Alliance.Red) ? ShooterConstants.SHOOT_POSE_RED : ShooterConstants.SHOOT_POSE_BLUE;
                // ShotParameters shootAimingParams =
                //         calculateAimingParameters(shootTargetPose, robotPose, robotSpeeds, false);
                // targetHoodAngle = Optional.of(shootAimingParams.hoodAngle);
                // targetTurretPosition = Optional.of(shootAimingParams.TurretAngle);
                // targetWheelSpeed = Optional.of(shootAimingParams.wheelSpeed);
                break;
            case SPINUP:
                targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                targetHoodAngle = Optional.of(Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES));
                break;
            case TEST:
                ShotParametersAutoLogged shot = shotCalculator.calculateShot(
                        robotPose,
                        new Translation3d(testTargetX.get(), testTargetY.get(), 0),
                        (double) (System.currentTimeMillis()),
                        0);
                Logger.processInputs("ShooterSubsystem/Shot", shot);
                Logger.recordOutput(
                        "ShooterSubsystem/TestTargetPose",
                        new Pose2d(testTargetX.get(), testTargetY.get(), new Rotation2d()));
                targetHoodAngle = Optional.of(shot.hoodAngle);
                targetTurretPosition = Optional.of(shot.turretAngle);

                break;
            case ZERO:
                hoodMotor.setVoltage(-hoodZeroVoltage.get());
                if (hoodMotor.getStatorCurrent().getValueAsDouble() >= hoodZeroCurrent.get()
                        && Math.abs(hoodMotor.getVelocity().getValueAsDouble())
                                <= Units.degreesToRotations(hoodZeroVelTol.get())) {
                    targetState = State.OFF;
                    hoodMotor.setPosition(Units.degreesToRotations(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                }
                break;
            case SYSTEMS_TEST:
                systemTestCount++;
                targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                targetTurretPosition =
                        Optional.of((((Math.sin(((double) systemTestCount / 850) + 1)) / 2) * 520) - 260);
                targetHoodAngle = Optional.of((((Math.sin(((double) systemTestCount / 150) + 1)) / 2) * 35) + 10);
        }

        if (targetHoodAngle.isPresent()) {
            double targetHoodAngleClamped = MathUtil.clamp(
                    targetHoodAngle.get(),
                    Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES),
                    Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MAX_ANGLE_DEGREES));
            double pidOutput = hoodController.calculate(currentHoodAngle, targetHoodAngleClamped);
            double hoodOutput = pidOutput;
            if (!hoodController.atSetpoint()) hoodMotor.setVoltage(hoodOutput);
            Logger.recordOutput("ShooterSubsystem/TargetHoodAngle", Units.radiansToDegrees(targetHoodAngle.get()));
            Logger.recordOutput("ShooterSubsystem/HoodOutput", hoodOutput);
            Logger.recordOutput("ShooterSubsystem/PidOutput", pidOutput);
        } else {
            if (targetState != State.ZERO) hoodMotor.setVoltage(0);
        }

        if (targetWheelSpeed.isPresent()) {
            double wheelOutput;
            Logger.recordOutput("Shooter/RPMError", getRPMError());
            if (getRPMError() > bangTol.get()) {
                wheelOutput = 12.0;
            } else {
                wheelOutput = wheelFF.calculate(targetWheelSpeed.get());
            }
            leftWheelMotor.setVoltage(wheelOutput);
            Logger.recordOutput("ShooterSubsystem/WheelOutput", wheelOutput);
            Logger.recordOutput("ShooterSubsystem/TargetWheelSpeed", targetWheelSpeed.get());
        } else {
            leftWheelMotor.setVoltage(0);
        }

        if (targetTurretPosition.isPresent()) {
            double normalizedTurretPosition = calculateTurretSetpoint(
                    Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble()), targetTurretPosition.get());
            double turretOutput = turretController.calculate(
                    Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble()), normalizedTurretPosition);

            Transform2d targetTurretTransform = new Transform2d(
                    ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(),
                    Rotation2d.fromRadians(normalizedTurretPosition));
            Pose2d targetTurretFieldPose = robotPose.toPose2d().transformBy(targetTurretTransform);

            turretMotor.setVoltage(turretOutput);
            Logger.recordOutput("ShooterSubsystem/TargetTurretPose", targetTurretFieldPose);
            Logger.recordOutput("ShooterSubsystem/TargetTurretPosition", targetTurretPosition.get());
            Logger.recordOutput("ShooterSubsystem/TurretOutput", turretOutput);
        } else {
            turretMotor.setVoltage(0);
        }

        Logger.recordOutput("ShooterSubsystem/CurrentHoodAngle", Units.radiansToDegrees(currentHoodAngle));
        Logger.recordOutput("ShooterSubsystem/CurrentTurretPosition", currentTurretPosition);
        Logger.recordOutput(
                "ShooterSubsystem/CurrentWheelSpeed", Units.radiansPerSecondToRotationsPerMinute(currentWheelSpeed));

        Logger.recordOutput("ShooterSubsystem/TurretPose", getFieldRelativeTurretPose());
        Logger.recordOutput(
                "ShooterSubsystem/HoodAmps", hoodMotor.getStatorCurrent().getValueAsDouble());

        Logger.recordOutput(
                "ShooterSubsystem/LeftMotorVolts",
                leftWheelMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(
                "ShooterSubsystem/RightMotorVolts",
                rightWheelMotor.getMotorVoltage().getValueAsDouble());
    }

    private double calculateTurretSetpoint(double currentAngle, double targetAngle) {
        double error = ((targetAngle - currentAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        if (error < 0) error += 2 * Math.PI;
        error -= Math.PI;

        double shortestPath = currentAngle + error;

        if (shortestPath > Math.toRadians(ShooterConstants.TURRENT_MAX_POSITION_DEGREES)) {
            return shortestPath - 2 * Math.PI;
        } else if (shortestPath < Math.toRadians(ShooterConstants.TURRENT_MIN_POSITION_DEGREES)) {
            return shortestPath + 2 * Math.PI;
        }
        return shortestPath;
    }

    private double getTurretRotation() {
        return Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble())
                + ShooterConstants.SHOOTER_OFFSET.getRotation().getZ();
    }
}
