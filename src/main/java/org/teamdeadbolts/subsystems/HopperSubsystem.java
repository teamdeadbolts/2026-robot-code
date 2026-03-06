/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.HopperConstants;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class HopperSubsystem extends StatefulSubsystem<HopperSubsystem.State> implements Refreshable {
    public enum State {
        HOLD,
        UP,
        DOWN;
    }

    private final CANBus canBus = new CANBus("*");
    private final TalonFX hopperMotorLeft = new TalonFX(HopperConstants.HOPPER_MOTOR_LEFT_CAN_ID, canBus);
    private final TalonFX hopperMotorRight = new TalonFX(HopperConstants.HOPPER_MOTOR_RIGHT_CAN_ID, canBus);

    private final PIDController lidLifterController = new PIDController(0.0, 0.0, 0.0);
    private final SimpleMotorFeedforward lidLifterFF = new SimpleMotorFeedforward(0, 0, 0);

    private final SavedLoggedNetworkNumber lidLifterControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/LidLifterController/kP", 0.1);

    private final SavedLoggedNetworkNumber lidLifterFFkS =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/LidLifterFF/kS", 0.0);

    private final SavedLoggedNetworkNumber lidDownHeight =
            SavedLoggedNetworkNumber.get("Tuning/Hopper/LidDownHeight", 0.0);
    private final SavedLoggedNetworkNumber lidUpHeight = SavedLoggedNetworkNumber.get("Tuning/Hopper/LidUpHeight", 0.0);

    private double holdHeight;

    public HopperSubsystem() {
        this.targetState = State.HOLD;
        this.holdHeight = lidDownHeight.get();
        hopperMotorRight.setControl(
                new Follower(HopperConstants.HOPPER_MOTOR_LEFT_CAN_ID, MotorAlignmentValue.Opposed));

        lidLifterControllerP.addRefreshable(this);
        lidLifterFFkS.addRefreshable(this);
    }

    @Override
    public void refresh() {
        HopperConstants.init();
        lidLifterController.setP(lidLifterControllerP.get());
        lidLifterFF.setKs(lidLifterFFkS.get());
        hopperMotorLeft.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
        hopperMotorRight.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
    }

    @Override
    protected void onStateChange(State to, State from) {
        lidLifterController.reset();

        if (to == State.HOLD) this.holdHeight = getLidHeight();
    }

    @Override
    public void periodic() {
        double targetHeight = 0;
        switch (this.targetState) {
            case HOLD:
                targetHeight = holdHeight;
                break;
            case UP:
                targetHeight = lidUpHeight.get();
                break;
            case DOWN:
                targetHeight = lidDownHeight.get();
                break;
        }

        double pidOutput = lidLifterController.calculate(getLidHeight(), targetHeight);
        double output = pidOutput + lidLifterFF.calculate(pidOutput); // Should be a trapizoid but probably chill
        hopperMotorLeft.setVoltage(output);

        // Logging
        Logger.recordOutput("HopperSubsystem/TargetHeight", targetHeight);
        Logger.recordOutput("HopperSubsystem/CurrentHeight", getLidHeight());
        Logger.recordOutput("HopperSubsystem/Output", output);
        Logger.recordOutput(
                "HopperSubsystem/RawMotorPosition",
                hopperMotorLeft.getPosition().getValueAsDouble());
    }

    private double getLidHeight() {
        return hopperMotorLeft.getPosition().getValueAsDouble() * HopperConstants.HOPPER_ROTATIONS_TO_METERS;
    }
}
