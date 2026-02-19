/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.teamdeadbolts.constants.HopperConstants;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class HopperSubsystem extends SubsystemBase {
    public enum State {
        HOLD,
        FAST_UP,
        FAST_DOWN,
        SLOW_UP,
        SLOW_DOWN,
    }

    @AutoLogOutput
    private State targetState = State.HOLD;

    private TalonFX hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_CAN_ID);
    private DigitalInput lowerLimitSwitch = new DigitalInput(HopperConstants.HOPPER_LOWER_LIMIT_SWITCH_CHANNEL);
    private DigitalInput upperLimitSwitch = new DigitalInput(HopperConstants.HOPPER_UPPER_LIMIT_SWITCH_CHANNEL);

    private final SavedLoggedNetworkNumber hopperMotorFastVolts =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorFastVolts", 1.0);
    private final SavedLoggedNetworkNumber hopperMotorSlowVolts =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorSlowVolts", 0.5);
    private final SavedLoggedNetworkNumber hopperMotorHoldVolts =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorHoldVolts", 0.0);

    public HopperSubsystem() {
        ConfigManager.getInstance().onReady(() -> {
            HopperConstants.init();
            hopperMotor.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
        });
    }

    public void setState(State newState) {
        targetState = newState;
    }

    public State getState() {
        return targetState;
    }

    @Override
    public void periodic() {
        switch (targetState) {
            case FAST_DOWN:
                hopperMotor.setVoltage(-hopperMotorFastVolts.get());
                if (lowerLimitSwitch.get()) {
                    targetState = State.HOLD;
                }
                break;
            case FAST_UP:
                hopperMotor.setVoltage(hopperMotorFastVolts.get());
                if (upperLimitSwitch.get()) {
                    targetState = State.HOLD;
                }
                break;
            case HOLD:
                hopperMotor.setVoltage(hopperMotorHoldVolts.get());
                break;
            case SLOW_DOWN:
                hopperMotor.setVoltage(-hopperMotorSlowVolts.get());
                if (lowerLimitSwitch.get()) {
                    targetState = State.HOLD;
                }
                break;
            case SLOW_UP:
                hopperMotor.setVoltage(hopperMotorSlowVolts.get());
                if (upperLimitSwitch.get()) {
                    targetState = State.HOLD;
                }
                break;
        }
    }
}
