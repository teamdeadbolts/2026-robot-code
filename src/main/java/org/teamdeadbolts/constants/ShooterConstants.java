/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShooterConstants {
    public static final int SHOOTER_TURRET_MOTOR_CAN_ID = 20;
    public static final int SHOOTER_HOOD_MOTOR_CAN_ID = 21;
    public static final int SHOOTER_WHEEL_MOTOR_LEFT_CAN_ID = 22;
    public static final int SHOOTER_WHEEL_MOTOR_RIGHT_CAN_ID = 23;

    public static final double SHOOTER_HOOD_MIN_ANGLE_DEGREES = 10.2;
    public static final double SHOOTER_HOOD_MAX_ANGLE_DEGREES = 43.0; // TODO fix!!

    public static final double TURRET_MIN_POSITION_DEGREES = -90.0;
    public static final double TURRET_MAX_POSITION_DEGREES = 270.0;

    public static final double WHEEL_MAX_OMEGA_RAD_PER_SEC = 600.0 * 2.0 * Math.PI / 60.0; // 600 RPM

    public static final double EXIT_SPEED_EFFICIENCY =
            0.95; // How well does the energy get transfered from the wheels to the ball

    public static final double SHOOTER_BIG_WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
    public static final double SHOOTER_SMALL_WHEEL_RADIUS_METERS = Units.inchesToMeters(1.0);
    public static final double EXIT_RADIUS_METERS = Units.inchesToMeters(4);

    public static final Pose3d PASS_TOP_POSE_RED = new Pose3d();
    public static final Pose3d PASS_TOP_POSE_BLUE = new Pose3d();
    public static final Pose3d PASS_BOTTOM_POSE_RED = new Pose3d();
    public static final Pose3d PASS_BOTTOM_POSE_BLUE = new Pose3d();
    public static final Pose3d SHOOT_POSE_RED = new Pose3d();
    public static final Pose3d SHOOT_POSE_BLUE = new Pose3d();

    public static final Transform3d SHOOTER_OFFSET = new Transform3d(
            Units.inchesToMeters(-27.5 / 2 + 6.125),
            Units.inchesToMeters(-27.5 / 2 + 6.125),
            Units.inchesToMeters(13.5),
            new Rotation3d(0.0, 0.0, Math.PI / 2));

    public static final double TURRET_GEAR_RATIO = ((145.0 / 15.0) * 3); // The gear ratio of the turret
    public static final double HOOD_GEAR_RATIO =
            (40.0 / 12.0) * (18.0 / 14.0) * (163.0 / 10.0); // The gear ratio of the hood
    public static final double WHEEL_GEAR_RATIO = 18.0 / 17.0; // The gear ratio of the wheels

    public static final TalonFXConfiguration SHOOTER_TURRET_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration SHOOTER_HOOD_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration SHOOTER_WHEEL_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final CANcoderConfiguration SHOOTER_ABS_ENCODER_CONFIG = new CANcoderConfiguration();

    public static final InterpolatingDoubleTreeMap SHOOTER_RPM_TO_MPS_MAP = new InterpolatingDoubleTreeMap();

    private static final SavedLoggedNetworkNumber shooterTurretMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterTurretMotorCurrentLimit", 40);
    private static final SavedLoggedNetworkNumber shooterHoodMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterHoodMotorCurrentLimit", 40);
    private static final SavedLoggedNetworkNumber shooterWheelMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Shooter/ShooterWheelMotorCurrentLimit", 80);

    static {
        ConfigManager.getInstance().onReady(ShooterConstants::init);
        SHOOTER_RPM_TO_MPS_MAP.put(2.902, 1000.0);
        SHOOTER_RPM_TO_MPS_MAP.put(4.151, 1500.0);
        SHOOTER_RPM_TO_MPS_MAP.put(6.773, 2000.0);
        SHOOTER_RPM_TO_MPS_MAP.put(7.175, 2500.0);
        SHOOTER_RPM_TO_MPS_MAP.put(8.709, 3000.0);
        SHOOTER_RPM_TO_MPS_MAP.put(9.562, 3500.0);
        SHOOTER_RPM_TO_MPS_MAP.put(10.16, 4000.0);
    }

    public static void init() {
        shooterTurretMotorCurrentLimit.initFromConfig();
        SHOOTER_TURRET_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_TURRET_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterTurretMotorCurrentLimit.get();
        SHOOTER_TURRET_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;
        SHOOTER_TURRET_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterHoodMotorCurrentLimit.initFromConfig();
        SHOOTER_HOOD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_HOOD_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterHoodMotorCurrentLimit.get();
        SHOOTER_HOOD_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        SHOOTER_HOOD_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        SHOOTER_HOOD_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterWheelMotorCurrentLimit.initFromConfig();
        SHOOTER_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_WHEEL_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = WHEEL_GEAR_RATIO;
        SHOOTER_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = shooterWheelMotorCurrentLimit.get();
        SHOOTER_WHEEL_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
}
