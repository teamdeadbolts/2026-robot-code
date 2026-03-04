/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.teamdeadbolts.constants.HopperConstants;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class HopperSubsystem extends SubsystemBase implements Refreshable {

    public enum State {
        HOLD,
        FAST_UP,
        FAST_DOWN,
        SLOW_UP,
        SLOW_DOWN,
    }

    @AutoLogOutput
    private State targetState = State.HOLD;

    private final CANBus canBus = new CANBus("*");
    private final TalonFX hopperMotorLeft = new TalonFX(HopperConstants.HOPPER_MOTOR_LEFT_CAN_ID, canBus);
    private final TalonFX hopperMotorRight = new TalonFX(HopperConstants.HOPPER_MOTOR_RIGHT_CAN_ID, canBus);
    private final DigitalInput lowerLimitSwitchLeft =
            new DigitalInput(HopperConstants.HOPPER_LEFT_LOWER_LIMIT_SWITCH_CHANNEL);
    private final DigitalInput upperLimitSwitchLeft =
            new DigitalInput(HopperConstants.HOPPER_LEFT_UPPER_LIMIT_SWITCH_CHANNEL);
    private final DigitalInput lowerLimitSwitchRight =
            new DigitalInput(HopperConstants.HOPPER_RIGHT_LOWER_LIMIT_SWITCH_CHANNEL);
    private final DigitalInput upperLimitSwitchRight =
            new DigitalInput(HopperConstants.HOPPER_RIGHT_UPPER_LIMIT_SWITCH_CHANNEL);

    private final SavedLoggedNetworkNumber hopperMotorFastVolts =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorFastVolts", 1.0);
    private final SavedLoggedNetworkNumber hopperMotorSlowVolts =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorSlowVolts", 0.5);
    private final SavedLoggedNetworkNumber hopperMotorHoldVolts =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/HopperMotorHoldVolts", 0.0);

    public HopperSubsystem() {
        ConfigManager.getInstance().onReady(this::refresh);
        hopperMotorRight.setControl(
                new Follower(HopperConstants.HOPPER_MOTOR_LEFT_CAN_ID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void refresh() {
        HopperConstants.init();
        hopperMotorLeft.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
        hopperMotorRight.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
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
                hopperMotorLeft.setVoltage(-hopperMotorFastVolts.get());
                if (lowerLimitSwitchLeft.get() || lowerLimitSwitchRight.get()) {
                    targetState = State.HOLD;
                }
                break;
            case FAST_UP:
                hopperMotorLeft.setVoltage(hopperMotorFastVolts.get());
                if (upperLimitSwitchLeft.get() || upperLimitSwitchRight.get()) {
                    targetState = State.HOLD;
                }
                break;
            case HOLD:
                hopperMotorLeft.setVoltage(hopperMotorHoldVolts.get());
                break;
            case SLOW_DOWN:
                hopperMotorLeft.setVoltage(-hopperMotorSlowVolts.get());
                if (lowerLimitSwitchLeft.get() || lowerLimitSwitchRight.get()) {
                    targetState = State.HOLD;
                }
                break;
            case SLOW_UP:
                hopperMotorLeft.setVoltage(hopperMotorSlowVolts.get());
                if (upperLimitSwitchLeft.get() || upperLimitSwitchRight.get()) {
                    targetState = State.HOLD;
                }
                break;
        }
    }

    public boolean isUpperLimitReached() {
        return upperLimitSwitchLeft.get() || upperLimitSwitchRight.get();
    }

    public boolean isLowerLimitReached() {
        return lowerLimitSwitchLeft.get() || lowerLimitSwitchRight.get();
    }
}
