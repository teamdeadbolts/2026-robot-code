/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class IntakeConstants {
    public static final int INTAKE_ARM_MOTOR_CAN_ID = 41;
    public static final int INTAKE_DRIVE_MOTOR_CAN_ID = 42;
    public static final int INTAKE_ABS_ENCODER_CAN_ID = 43;

    public static final TalonFXConfiguration INTAKE_ARM_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration INTAKE_WHEEL_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final CANcoderConfiguration INTAKE_ABS_ENCODER_CONFIG = new CANcoderConfiguration();

    private static final SavedLoggedNetworkNumber intakeArmMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeArmMotorCurrentLimit", 20);
    private static final SavedLoggedNetworkNumber intakeWheelMotorCurrentLimit =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeWheelMotorCurrentLimit", 20);

    static {
        ConfigManager.getInstance().onReady(IntakeConstants::init);
    }

    public static void init() {
        intakeArmMotorCurrentLimit.initFromConfig();
        INTAKE_ARM_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        INTAKE_ARM_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = intakeArmMotorCurrentLimit.get();
        INTAKE_ARM_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        INTAKE_ARM_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeWheelMotorCurrentLimit.initFromConfig();
        INTAKE_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
        INTAKE_WHEEL_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = intakeWheelMotorCurrentLimit.get();
    }
}
