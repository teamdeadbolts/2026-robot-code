/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class HopperConstants {
    public static final int HOPPER_MOTOR_LEFT_CAN_ID = 60;
    public static final int HOPPER_MOTOR_RIGHT_CAN_ID = 61;
    public static final int HOPPER_LEFT_UPPER_LIMIT_SWITCH_CHANNEL = -1;
    public static final int HOPPER_LEFT_LOWER_LIMIT_SWITCH_CHANNEL = -1;
    public static final int HOPPER_RIGHT_UPPER_LIMIT_SWITCH_CHANNEL = -1;
    public static final int HOPPER_RIGHT_LOWER_LIMIT_SWITCH_CHANNEL = -1;

    public static final double HOPPER_LEFT_ROTATIONS_TO_METERS = 4 / 43.3415; // Rotations/meter
    public static final double HOPPER_RIGHT_ROTATIONS_TO_METERS = (4 / 43.3415) / 1.174; // Rotations/meter

    public static final TalonFXConfiguration LEFT_HOPPER_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration RIGHT_HOPPER_MOTOR_CONFIG = new TalonFXConfiguration();

    private static SavedLoggedNetworkNumber lidLifterCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/LidLifterCurrentLimit", 20);

    static {
        ConfigManager.getInstance().onReady(HopperConstants::init);
    }

    public static void init() {
        lidLifterCurrentLimit.initFromConfig();
        LEFT_HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        LEFT_HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = lidLifterCurrentLimit.get();
        LEFT_HOPPER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        LEFT_HOPPER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        RIGHT_HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        RIGHT_HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = lidLifterCurrentLimit.get();
        RIGHT_HOPPER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        RIGHT_HOPPER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}
