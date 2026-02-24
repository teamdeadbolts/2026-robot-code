/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import org.teamdeadbolts.subsystems.drive.SwerveModule.SwerveModuleConfig;
import org.teamdeadbolts.utils.Zone;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class SwerveConstants {
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2 * 2.25 * Math.PI);
    public static final double CHASSIS_SIZE = Units.inchesToMeters(27.5);
    public static final SensorDirectionValue SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    /* Turning constants */
    public static final InvertedValue TURN_INVERTED_MODE = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue TURN_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double TURN_GEAR_RATIO = 287 / 11;

    /* Driving constants */
    public static final InvertedValue DRIVE_INVERTED_MODE = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final double DRIVE_GEAR_RATIO_R1 = 7.03;
    public static final double DRIVE_GEAR_RATIO_R2 = 6.03;
    public static final double DRIVE_GEAR_RATIO_R3 = 5.27;

    /* Module constants */
    public static final SwerveModuleConfig FRONT_LEFT_CONFIG =
            new SwerveModuleConfig(0, Rotation2d.fromDegrees(-74.883 + 180), 0, 1, 2);
    public static final SwerveModuleConfig FRONT_RIGHT_CONFIG =
            new SwerveModuleConfig(1, Rotation2d.fromDegrees(99.668), 3, 4, 5);
    public static final SwerveModuleConfig BACK_LEFT_CONFIG =
            new SwerveModuleConfig(2, Rotation2d.fromDegrees(140.977), 6, 7, 8);
    public static final SwerveModuleConfig BACK_RIGHT_CONFIG =
            new SwerveModuleConfig(3, Rotation2d.fromDegrees(125.068), 9, 10, 11);

    /* Kinematics */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(CHASSIS_SIZE / 2.0, CHASSIS_SIZE / 2.0), // Front Left (B)
            new Translation2d(CHASSIS_SIZE / 2.0, -CHASSIS_SIZE / 2.0), // Front Right (A)
            new Translation2d(-CHASSIS_SIZE / 2.0, CHASSIS_SIZE / 2.0), // Back Left (B)
            new Translation2d(-CHASSIS_SIZE / 2.0, -CHASSIS_SIZE / 2.0)); // Back Right (A)

    // TODO: Find these
    public static final Zone BLUE_TOP_BUMP_ZONE = new Zone();
    public static final Zone BLUE_BOTTOM_BUMP_ZONE = new Zone();
    public static final Zone RED_TOP_BUMP_ZONE = new Zone();
    public static final Zone RED_BOTTOM_BUMP_ZONE = new Zone();

    public static final Zone RED_SCORE_ZONE = new Zone();
    public static final Zone BLUE_SCORE_ZONE = new Zone();

    public static final Zone RED_CLOSE_ZONE = new Zone();
    public static final Zone BLUE_CLOSE_ZONE = new Zone();

    public static final TalonFXConfiguration TURNING_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration DRIVE_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final CANcoderConfiguration CANCODER_CONFIG = new CANcoderConfiguration();

    private static final SavedLoggedNetworkNumber tCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Swerve/Turn/CurrentLimit", 0.0);

    private static final SavedLoggedNetworkNumber dCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Swerve/Drive/CurrentLimit", 0.0);

    static {
        ConfigManager.getInstance().onReady(SwerveConstants::init);
    }

    public static void init() {
        tCurrentLimit.initFromConfig();
        dCurrentLimit.initFromConfig();

        TURNING_MOTOR_CONFIG.MotorOutput.Inverted = TURN_INVERTED_MODE;
        TURNING_MOTOR_CONFIG.MotorOutput.NeutralMode = TURN_NEUTRAL_MODE;

        TURNING_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;
        TURNING_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

        TURNING_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        TURNING_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = tCurrentLimit.get();

        DRIVE_MOTOR_CONFIG.MotorOutput.Inverted = DRIVE_INVERTED_MODE;
        DRIVE_MOTOR_CONFIG.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

        DRIVE_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO_R2;

        DRIVE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        DRIVE_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = dCurrentLimit.get();

        CANCODER_CONFIG.MagnetSensor.SensorDirection = SENSOR_DIRECTION;
    }
}
