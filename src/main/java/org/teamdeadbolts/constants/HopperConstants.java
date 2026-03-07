/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    public static final double HOPPER_ROTATIONS_TO_METERS = 1; // TODO: measure

    public static final TalonFXConfiguration HOPPER_MOTOR_CONFIG = new TalonFXConfiguration();

    private static SavedLoggedNetworkNumber hopperMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorCurrentLimit", 20);

    static {
        ConfigManager.getInstance().onReady(HopperConstants::init);
    }

    public static void init() {
        hopperMotorCurrentLimit.initFromConfig();
        HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        HOPPER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = hopperMotorCurrentLimit.get();
        HOPPER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
}
