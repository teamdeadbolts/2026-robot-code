/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.teamdeadbolts.utils.tuning.ConfigManager;

public class ClimberConstants {
    public static final int CLIMBER_MOTOR_CAN_ID = -1;
    public static final TalonFXConfiguration CLIMBER_MOTOR_CONFIG = new TalonFXConfiguration();

    public static final double CLIMBER_GEAR_RATIO = 1;

    static {
        ConfigManager.getInstance().onReady(ClimberConstants::init);
    }

    public static void init() {
        CLIMBER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = CLIMBER_GEAR_RATIO;
    }
}
