/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import org.teamdeadbolts.subsystems.drive.SwerveModule.SwerveModuleConfig;
import org.teamdeadbolts.utils.Zone;

public class SwerveConstants {
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2 * 2.25 * Math.PI);
    public static final double CHASSIS_SIZE = Units.inchesToMeters(30);
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
            new SwerveModuleConfig(0, Rotation2d.fromDegrees(51.8), 0, 1, 2);
    public static final SwerveModuleConfig FRONT_RIGHT_CONFIG =
            new SwerveModuleConfig(1, Rotation2d.fromDegrees(105.5 - 90), 3, 4, 5);
    public static final SwerveModuleConfig BACK_LEFT_CONFIG =
            new SwerveModuleConfig(2, Rotation2d.fromDegrees(-56.3 + 90), 6, 7, 8);
    public static final SwerveModuleConfig BACK_RIGHT_CONFIG =
            new SwerveModuleConfig(3, Rotation2d.fromDegrees(-171.7 + 180), 9, 10, 11);

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
}
