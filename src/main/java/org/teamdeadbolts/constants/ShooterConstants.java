/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShooterConstants {
    public static final int SHOOTER_TURRET_MOTOR_CAN_ID = 20;
    public static final int SHOOTER_HOOD_MOTOR_CAN_ID = 21;
    public static final int SHOOTER_WHEEL_MOTOR_CAN_ID = 22;

    public static final double SHOOTER_HOOD_MIN_ANGLE_DEGREES = 15.0;
    public static final double SHOOTER_HOOD_MAX_ANGLE_DEGREES = 45.0;

    public static final double TURRENT_MIN_POSITION_DEGREES = -270.0;
    public static final double TURRENT_MAX_POSITION_DEGREES = 270.0;

    public static final double WHEEL_MAX_OMEGA_RAD_PER_SEC = 600.0 * 2.0 * Math.PI / 60.0; // 600 RPM

    public static final double EXIT_SPEED_EFFICIENCY = 0.95;

    public static final double SHOOTER_WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

    public static final ExitOffset[] EXIT_OFFSETS = new ExitOffset[] {}; // TOOD

    public static record ExitOffset(double hoodRad, double x, double y, double z) {}

    public static final Pose3d PASS_LEFT_POSE_RED = new Pose3d();
    public static final Pose3d PASS_RIGHT_POSE_BLUE = new Pose3d();
    public static final Pose3d PASS_RIGHT_POSE_RED = new Pose3d();
    public static final Pose3d PASS_LEFT_POSE_BLUE = new Pose3d();
    public static final Pose3d SHOOT_POSE_RED = new Pose3d();
    public static final Pose3d SHOOT_POSE_BLUE = new Pose3d();

    public static final Transform3d SHOOTER_OFFSET = new Transform3d(
            Units.inchesToMeters(-15 + 6.125),
            Units.inchesToMeters(-15 + 6.125),
            Units.inchesToMeters(13.5),
            new Rotation3d());

    public static final double TURRENT_GEAR_RATIO = (145.0 / 18.0) * 3;
    public static final double HOOD_GEAR_RATIO = (40.0 / 12.0) * (25.0 / 13.0) * (168.0 / 10.0);

    public static final TalonFXConfiguration SHOOTER_TURRET_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration SHOOTER_HOOD_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration SHOOTER_WHEEL_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final CANcoderConfiguration SHOOTER_ABS_ENCODER_CONFIG = new CANcoderConfiguration();

    private static final SavedLoggedNetworkNumber shooterTurretMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterTurretMotorCurrentLimit", 20);
    private static final SavedLoggedNetworkNumber shooterHoodMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterHoodMotorCurrentLimit", 20);
    private static final SavedLoggedNetworkNumber shooterWheelMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterWheelMotorCurrentLimit", 20);

    static {
        ConfigManager.getInstance().onReady(ShooterConstants::init);
    }

    public static void init() {
        shooterTurretMotorCurrentLimit.initFromConfig();
        SHOOTER_TURRET_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_TURRET_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterTurretMotorCurrentLimit.get();
        SHOOTER_TURRET_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = TURRENT_GEAR_RATIO;
        SHOOTER_TURRET_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterHoodMotorCurrentLimit.initFromConfig();
        SHOOTER_HOOD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_HOOD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterHoodMotorCurrentLimit.get();
        SHOOTER_HOOD_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        SHOOTER_HOOD_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        SHOOTER_HOOD_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // SHOOTER_HOOD_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        SHOOTER_HOOD_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        SHOOTER_HOOD_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.degreesToRotations(SHOOTER_HOOD_MAX_ANGLE_DEGREES);
        SHOOTER_HOOD_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        SHOOTER_HOOD_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.degreesToRotations(SHOOTER_HOOD_MIN_ANGLE_DEGREES);

        shooterWheelMotorCurrentLimit.initFromConfig();
        SHOOTER_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterWheelMotorCurrentLimit.get();
        SHOOTER_WHEEL_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
}
