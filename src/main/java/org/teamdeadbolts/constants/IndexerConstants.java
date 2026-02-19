/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class IndexerConstants {
    public static final int INDEXER_FLOOR_MOTOR_CAN_ID = -1;
    public static final int INDEXER_KICKER_MOTOR_CAN_ID = 31;
    public static final int INDEXER_BALL_SENSOR_CHANNEL = -1;

    public static final TalonFXConfiguration INDEXER_FLOOR_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration INDEXER_KICKER_MOTOR_CONFIG = new TalonFXConfiguration();

    private static SavedLoggedNetworkNumber indexerFloorMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Indexer/IndexerFloorMotorCurrentLimit", 20);
    private static SavedLoggedNetworkNumber indexerKickerMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Indexer/IndexerKickerMotorCurrentLimit", 20);

    static {
        ConfigManager.getInstance().onReady(IndexerConstants::init);
    }

    public static void init() {
        indexerFloorMotorCurrentLimit.initFromConfig();
        INDEXER_FLOOR_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        INDEXER_FLOOR_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = indexerFloorMotorCurrentLimit.get();

        indexerKickerMotorCurrentLimit.initFromConfig();
        INDEXER_KICKER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        INDEXER_KICKER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = indexerKickerMotorCurrentLimit.get();
        INDEXER_KICKER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
}
