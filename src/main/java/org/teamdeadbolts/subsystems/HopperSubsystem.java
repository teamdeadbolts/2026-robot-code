/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.HopperConstants;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Manages the hopper subsystem,to position the lid (maybe we should call it lid subsystem)
 * Uses PID control to maintain the lid at specified heights.
 */
public class HopperSubsystem extends StatefulSubsystem<HopperSubsystem.State> implements Refreshable {
    public enum State {
        HOLD,
        UP,
        DOWN;
    }

    private final CANBus canBus = new CANBus("*");
    private final TalonFX hopperMotorLeft = new TalonFX(HopperConstants.HOPPER_MOTOR_LEFT_CAN_ID, canBus);
    private final TalonFX hopperMotorRight = new TalonFX(HopperConstants.HOPPER_MOTOR_RIGHT_CAN_ID, canBus);

    private final PIDController leftLifterController = new PIDController(0.0, 0.0, 0.0);
    private final SimpleMotorFeedforward leftLifterFF = new SimpleMotorFeedforward(0, 0, 0);

    private final PIDController rightLifterController = new PIDController(0.0, 0.0, 0.0);
    private final SimpleMotorFeedforward rightLifterFF = new SimpleMotorFeedforward(0, 0, 0);

    /* --- Tuning Parameters --- */
    private final SavedLoggedNetworkNumber leftLifterControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/LeftLifterController/kP", 0.1);
    private final SavedLoggedNetworkNumber leftLifterFFkS =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/LeftLifterFF/kS", 0.0);

    private final SavedLoggedNetworkNumber rightLifterControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/RightLifterController/kP", 0.1);
    private final SavedLoggedNetworkNumber rightLifterFFkS =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/RightLifterFF/kS", 0.0);

    private final SavedLoggedNetworkNumber lidDownHeight =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/LidDownHeight", 0.0);
    private final SavedLoggedNetworkNumber lidUpHeight = SavedLoggedNetworkNumber.get("Tuning/Hopper/LidUpHeight", 0.0);

    private double holdHeight;

    public HopperSubsystem() {
        this.targetState = State.HOLD;
        this.holdHeight = lidDownHeight.get();
        hopperMotorLeft.setPosition(0);
        hopperMotorRight.setPosition(0);

        // Configure right motor to follow left motor in opposition
        // hopperMotorRight.setControl(
        //         new Follower(HopperConstants.HOPPER_MOTOR_LEFT_CAN_ID, MotorAlignmentValue.Aligned));

        leftLifterControllerP.addRefreshable(this);
        leftLifterFFkS.addRefreshable(this);
        rightLifterControllerP.addRefreshable(this);
        rightLifterFFkS.addRefreshable(this);
    }

    @Override
    public void refresh() {
        HopperConstants.init();
        leftLifterController.setP(leftLifterControllerP.get());
        leftLifterFF.setKs(leftLifterFFkS.get());

        rightLifterController.setP(rightLifterControllerP.get());
        rightLifterFF.setKs(rightLifterFFkS.get());

        hopperMotorLeft.getConfigurator().apply(HopperConstants.LEFT_HOPPER_MOTOR_CONFIG);
        hopperMotorRight.getConfigurator().apply(HopperConstants.RIGHT_HOPPER_MOTOR_CONFIG);
    }

    @Override
    protected void onStateChange(State to, State from) {
        leftLifterController.reset();
        rightLifterController.reset();

        if (to == State.HOLD) this.holdHeight = getLeftLidHeight();
    }

    @Override
    public void periodic() {
        double targetHeight =
                switch (this.targetState) {
                    case HOLD -> holdHeight;
                    case UP -> lidUpHeight.get();
                    case DOWN -> lidDownHeight.get();
                };

        // Calculate control effort
        double leftPidOutput = leftLifterController.calculate(getLeftLidHeight(), targetHeight);
        double leftOutput = leftPidOutput + leftLifterFF.calculate(leftPidOutput);
        hopperMotorLeft.setVoltage(leftOutput);

        double rightPidOutput = rightLifterController.calculate(getRightLidHeight(), targetHeight);
        double rightOutput = rightPidOutput + rightLifterFF.calculate(rightPidOutput);
        hopperMotorRight.setVoltage(rightOutput);

        // Logging
        Logger.recordOutput("HopperSubsystem/TargetHeight", targetHeight);
        Logger.recordOutput("HopperSubsystem/LeftCurrentHeight", getLeftLidHeight());
        Logger.recordOutput("HopperSubsystem/LeftOutput", leftOutput);
        Logger.recordOutput(
                "HopperSubsystem/LeftRawMotorPosition",
                hopperMotorLeft.getPosition().getValueAsDouble());

        Logger.recordOutput("HopperSubsystem/RightCurrentHeight", getRightLidHeight());
        Logger.recordOutput("HopperSubsystem/RightOutput", rightOutput);
        Logger.recordOutput(
                "HopperSubsystem/RightRawMotorPosition",
                hopperMotorRight.getPosition().getValueAsDouble());
    }

    /** @return The current lid height in meters. */
    private double getLeftLidHeight() {
        return hopperMotorLeft.getPosition().getValueAsDouble() * HopperConstants.HOPPER_LEFT_ROTATIONS_TO_METERS;
    }

    private double getRightLidHeight() {
        return hopperMotorRight.getPosition().getValueAsDouble() * HopperConstants.HOPPER_RIGHT_ROTATIONS_TO_METERS;
    }
}
