/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ShooterConstants;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {
    public enum State {
        OFF,
        SPINUP,
        SHOOT,
        PASS_LEFT,
        PASS_RIGHT,
        TEST;
    }

    @AutoLogOutput
    private State targetState = State.OFF;

    private TalonFX turretMotor = new TalonFX(ShooterConstants.SHOOTER_TURRET_MOTOR_CAN_ID);
    private TalonFX hoodMotor = new TalonFX(ShooterConstants.SHOOTER_HOOD_MOTOR_CAN_ID);
    private TalonFX wheelMotor = new TalonFX(ShooterConstants.SHOOTER_WHEEL_MOTOR_CAN_ID);

    private PIDController hoodController = new PIDController(0.0, 0.0, 0.0);
    private PIDController turretController = new PIDController(0.0, 0.0, 0.0);
    private PIDController wheelController = new PIDController(0.0, 0.0, 0.0);
    private SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(0, 0, 0);

    private SavedLoggedNetworkNumber hoodControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kP", 0.1);
    private SavedLoggedNetworkNumber hoodControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kI", 0.0);
    private SavedLoggedNetworkNumber hoodControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/HoodController/kD", 0.0);

    private SavedLoggedNetworkNumber turretControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kP", 0.1);
    private SavedLoggedNetworkNumber turretControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kI", 0.0);
    private SavedLoggedNetworkNumber turretControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TurretController/kD", 0.0);

    private SavedLoggedNetworkNumber wheelControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kP", 0.1);
    private SavedLoggedNetworkNumber wheelControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kI", 0.0);
    private SavedLoggedNetworkNumber wheelControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kD", 0.0);

    private SavedLoggedNetworkNumber wheelFFS = SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kS", 0.1);
    private SavedLoggedNetworkNumber wheelFFV = SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kV", 1);
    private SavedLoggedNetworkNumber wheelFFA = SavedLoggedNetworkNumber.get("Tuning/Shooter/WheelController/kA", 0.0);

    private SavedLoggedNetworkNumber shooterWheelSpinupSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterWheelSpinupSpeed", 5000.0); // RPM

    private SavedLoggedNetworkNumber testHoodAngle = SavedLoggedNetworkNumber.get("Tuning/Shooter/TestHoodAngle", 45);
    private SavedLoggedNetworkNumber testShooterRPM =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/TestShooterRPM", 2500);

    private SavedLoggedNetworkNumber testTargetX = SavedLoggedNetworkNumber.get("Tuning/Shooter/TestTargetX", 0);
    private SavedLoggedNetworkNumber testTargetY = SavedLoggedNetworkNumber.get("Tuning/Shooter/TestTargetY", 0);

    private Optional<Double> targetWheelSpeed;
    private double currentWheelSpeed;

    public ShooterSubsystem() {
        ConfigManager.getInstance().onReady(this::reconfigure);
        resetTurrentPosition();
        hoodMotor.setPosition(Units.degreesToRotations(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
        wheelFFS.onChange(wheelFF::setKs);
        wheelFFV.onChange(wheelFF::setKv);
        wheelFFA.onChange(wheelFF::setKa);
    }

    public void reconfigure() {
        hoodController.setPID(hoodControllerP.get(), hoodControllerI.get(), hoodControllerD.get());
        turretController.setPID(turretControllerP.get(), turretControllerI.get(), turretControllerD.get());
        wheelController.setPID(wheelControllerP.get(), wheelControllerI.get(), wheelControllerD.get());
        wheelFF.setKs(wheelFFS.get());
        wheelFF.setKv(wheelFFV.get());
        wheelFF.setKa(wheelFFA.get());

        ShooterConstants.init();
        hoodMotor.getConfigurator().apply(ShooterConstants.SHOOTER_HOOD_MOTOR_CONFIG);
        turretMotor.getConfigurator().apply(ShooterConstants.SHOOTER_TURRET_MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(ShooterConstants.SHOOTER_WHEEL_MOTOR_CONFIG);
    }

    public void setState(State newState) {
        targetState = newState;
    }

    public State getState() {
        return targetState;
    }

    public double getRPMError() {
        if (targetWheelSpeed.isEmpty()) return 0;
        return targetWheelSpeed.get() - currentWheelSpeed;
    }

    public void resetTurrentPosition() {
        this.turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        double currentHoodAngle =
                Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble());
        Optional<Double> targetHoodAngle = Optional.empty();

        double currentTurrentPosition =
                Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble());
        Optional<Double> targetTurretPosition = Optional.empty();

        currentWheelSpeed = Units.rotationsToRadians(wheelMotor.getVelocity().getValueAsDouble());
        targetWheelSpeed = Optional.empty();

        Pose3d robotPose = RobotState.getInstance().getRobotPose();
        ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeRobotVelocities();

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        switch (targetState) {
            case OFF:
                targetHoodAngle = Optional.of(Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                // targetWheelSpeed = Optional.
                break;
            case PASS_LEFT:
            case PASS_RIGHT:
                Pose3d targetPose;

                if (alliance == Alliance.Red) {
                    targetPose = targetState == State.PASS_LEFT
                            ? ShooterConstants.PASS_LEFT_POSE_RED
                            : ShooterConstants.PASS_RIGHT_POSE_RED;
                } else {
                    targetPose = targetState == State.PASS_LEFT
                            ? ShooterConstants.PASS_LEFT_POSE_BLUE
                            : ShooterConstants.PASS_RIGHT_POSE_BLUE;
                }

                // ShotParameters aimingParams =
                //         calculateAimingParameters(targetPose, robotPose, robotSpeeds, true);
                // targetHoodAngle = Optional.of(aimingParams.hoodAngle);
                // targetTurretPosition = Optional.of(aimingParams.turrentAngle);
                // targetWheelSpeed = Optional.of(aimingParams.wheelSpeed);
                break;
            case SHOOT:
                Pose3d shootTargetPose =
                        (alliance == Alliance.Red) ? ShooterConstants.SHOOT_POSE_RED : ShooterConstants.SHOOT_POSE_BLUE;
                // ShotParameters shootAimingParams =
                //         calculateAimingParameters(shootTargetPose, robotPose, robotSpeeds, false);
                // targetHoodAngle = Optional.of(shootAimingParams.hoodAngle);
                // targetTurretPosition = Optional.of(shootAimingParams.turrentAngle);
                // targetWheelSpeed = Optional.of(shootAimingParams.wheelSpeed);
                break;
            case SPINUP:
                targetWheelSpeed = Optional.of(shooterWheelSpinupSpeed.get());
                targetHoodAngle = Optional.of(Units.degreesToRadians(ShooterConstants.SHOOTER_HOOD_MIN_ANGLE_DEGREES));
                break;
            case TEST:
                // targetTurretPosition = Optional.of(calculateFieldRelativeTurrent(new Translation2d(2.836, 0.016)));
                ShotParametersAutoLogged shot = ShotCalculator.calculateShot(
                        robotPose, new Translation3d(testTargetX.get(), testTargetY.get(), 0));
                Logger.processInputs("ShooterSubsystem/Shot", shot);
                Logger.recordOutput(
                        "ShooterSubsystem/TestTargetPose",
                        new Pose2d(testTargetX.get(), testTargetY.get(), new Rotation2d()));
                targetHoodAngle = Optional.of(shot.hoodAngle);
                targetTurretPosition = Optional.of(shot.turrentAngle);
                targetWheelSpeed = Optional.of(shot.wheelSpeed);

                break;
        }

        if (targetHoodAngle.isPresent()) {
            double hoodOutput = hoodController.calculate(currentHoodAngle, targetHoodAngle.get());
            hoodMotor.setVoltage(hoodOutput);
            Logger.recordOutput("ShooterSubsystem/TargetHoodAngle", Units.radiansToDegrees(targetHoodAngle.get()));
            Logger.recordOutput("ShooterSubsystem/HoodOutput", hoodOutput);
        } else {
            hoodMotor.setVoltage(0);
        }

        if (targetWheelSpeed.isPresent()) {
            double wheelOutput = wheelFF.calculate(targetWheelSpeed.get())
                    + wheelController.calculate(currentWheelSpeed, targetWheelSpeed.get());
            wheelMotor.setVoltage(wheelOutput);
            Logger.recordOutput("ShooterSubsystem/WheelOutput", wheelOutput);
            Logger.recordOutput("ShooterSubsystem/TargetWheelSpeed", targetWheelSpeed.get());
        } else {
            wheelMotor.setVoltage(0);
        }

        if (targetTurretPosition.isPresent()) {
            double normalizedTurretPosition =
                    calculateTurrentSetpoint(currentTurrentPosition, targetTurretPosition.get());
            double turretOutput = turretController.calculate(currentTurrentPosition, normalizedTurretPosition);

            Transform2d targetTurrentTransform = new Transform2d(
                    ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(),
                    Rotation2d.fromRadians(normalizedTurretPosition));
            Pose2d targetTurretFieldPose = robotPose.toPose2d().transformBy(targetTurrentTransform);

            turretMotor.setVoltage(turretOutput);
            Logger.recordOutput("ShooterSubsystem/TargetTurretPose", targetTurretFieldPose);
            Logger.recordOutput("ShooterSubsystem/TargetTurretPosition", targetTurretPosition.get());
            Logger.recordOutput("ShooterSubsystem/TurrentOutput", turretOutput);
        } else {
            turretMotor.setVoltage(0);
        }

        Logger.recordOutput("ShooterSubsystem/CurrentHoodAngle", Units.radiansToDegrees(currentHoodAngle));
        Logger.recordOutput("ShooterSubsystem/CurrentTurretPosition", currentTurrentPosition);
        Logger.recordOutput(
                "ShooterSubsystem/CurrentWheelSpeed", Units.radiansPerSecondToRotationsPerMinute(currentWheelSpeed));

        Transform2d turretOffsetWithRotation = new Transform2d(
                ShooterConstants.SHOOTER_OFFSET.getTranslation().toTranslation2d(),
                Rotation2d.fromRadians(currentTurrentPosition));

        Pose2d turretFieldPose = robotPose.toPose2d().transformBy(turretOffsetWithRotation);

        Logger.recordOutput("ShooterSubsystem/TurretPose", turretFieldPose);
    }

    private double calculateTurrentSetpoint(double currentAngle, double targetAngle) {
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
}
