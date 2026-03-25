/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.HopperConstants;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.utils.PeriodicTasks;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

/**
 * Manages the hopper subsystem,to position the lid (maybe we should call it lid subsystem)
 * Uses PID control to maintain the lid at specified heights.
 */
public class HopperSubsystem extends StatefulSubsystem<HopperSubsystem.State> implements Refreshable {
    public enum State {
        UP,
        DOWN
    }

    private final CANBus canBus = new CANBus("*");
    private final TalonFX hopperMotorLeft = new TalonFX(HopperConstants.HOPPER_MOTOR_LEFT_CAN_ID, canBus);
    private final TalonFX hopperMotorRight = new TalonFX(HopperConstants.HOPPER_MOTOR_RIGHT_CAN_ID, canBus);

    private final PIDController leftLifterController = new PIDController(0.0, 0.0, 0.0);
    private final SimpleMotorFeedforward leftLifterFF = new SimpleMotorFeedforward(0, 0, 0);

    private final PIDController rightLifterController = new PIDController(0.0, 0.0, 0.0);
    private final SimpleMotorFeedforward rightLifterFF = new SimpleMotorFeedforward(0, 0, 0);

    private final StatusSignal<Angle> leftLifterPositionSignal = hopperMotorLeft.getPosition();
    private final StatusSignal<Angle> rightLifterPositionSignal = hopperMotorRight.getPosition();
    private final StatusSignal<Current> leftLifterCurrentSignal = hopperMotorLeft.getSupplyCurrent();
    private final StatusSignal<Current> rightLifterCurrentSignal = hopperMotorRight.getSupplyCurrent();

    private final List<BaseStatusSignal> signals = List.of(
            leftLifterPositionSignal, rightLifterPositionSignal, leftLifterCurrentSignal, rightLifterCurrentSignal);

    /* --- Tuning Parameters --- */
    private final SavedTunableNumber leftLifterControllerP =
            SavedTunableNumber.get("Tuning/Hopper/LeftLifterController/kP", 0.1);
    private final SavedTunableNumber leftLifterControllerI =
            SavedTunableNumber.get("Tuning/Hopper/LeftLifterController/kI", 0.1);
    private final SavedTunableNumber leftLifterFFkS = SavedTunableNumber.get("Tuning/Hopper/LeftLifterFF/kS", 0.0);

    private final SavedTunableNumber rightLifterControllerP =
            SavedTunableNumber.get("Tuning/Hopper/RightLifterController/kP", 0.1);
    private final SavedTunableNumber rightLifterControllerI =
            SavedTunableNumber.get("Tuning/Hopper/RightLifterController/kI", 0.1);
    private final SavedTunableNumber rightLifterFFkS = SavedTunableNumber.get("Tuning/Hopper/RightLifterFF/kS", 0.0);

    private final SavedTunableNumber lifterIZone = SavedTunableNumber.get("Tuning/Hopper/LifterIZone", 0.0);
    private final SavedTunableNumber lifterMaxI = SavedTunableNumber.get("Tuning/Hopper/LifterMaxI", 0.0);

    private final SavedTunableNumber lifterTol = SavedTunableNumber.get("Tuning/Hopper/LifterTol", 0.0);

    private final SavedTunableNumber lidDownHeight = SavedTunableNumber.get("Tuning/Hopper/LidDownHeight", 0.0);
    private final SavedTunableNumber lidUpHeight = SavedTunableNumber.get("Tuning/Hopper/LidUpHeight", 0.0);

    public HopperSubsystem() {
        super(State.DOWN);
        hopperMotorLeft.setPosition(0);
        hopperMotorRight.setPosition(0);

        lifterTol.addRefreshable(this);
        leftLifterControllerP.addRefreshable(this);
        leftLifterControllerP.addRefreshable(this);
        leftLifterFFkS.addRefreshable(this);
        rightLifterControllerP.addRefreshable(this);
        rightLifterControllerI.addRefreshable(this);
        rightLifterFFkS.addRefreshable(this);

        lifterIZone.addRefreshable(this);
        lifterMaxI.addRefreshable(this);
    }

    @Override
    public void refresh() {
        HopperConstants.init();
        leftLifterController.setP(leftLifterControllerP.get());
        leftLifterController.setI(leftLifterControllerI.get());
        leftLifterFF.setKs(leftLifterFFkS.get());

        rightLifterController.setP(rightLifterControllerP.get());
        rightLifterController.setI(rightLifterControllerI.get());
        rightLifterFF.setKs(rightLifterFFkS.get());

        leftLifterController.setIZone(lifterIZone.get());
        leftLifterController.setIntegratorRange(0, lifterMaxI.get());

        rightLifterController.setIZone(lifterIZone.get());
        rightLifterController.setIntegratorRange(0, lifterMaxI.get());

        rightLifterController.setTolerance(lifterTol.get());
        leftLifterController.setTolerance(lifterTol.get());

        hopperMotorLeft.getConfigurator().apply(HopperConstants.LEFT_HOPPER_MOTOR_CONFIG);
        hopperMotorRight.getConfigurator().apply(HopperConstants.RIGHT_HOPPER_MOTOR_CONFIG);
    }

    public boolean lidAtGoal() {
        return leftLifterController.atSetpoint() && rightLifterController.atSetpoint();
    }

    @Override
    protected void onStateChange(final State to, final State from) {
        leftLifterController.reset();
        rightLifterController.reset();
    }

    public List<BaseStatusSignal> getSignals() {
        return signals;
    }

    @Override
    public void subsystemPeriodic() {
        if (PeriodicTasks.getInstance().shouldRefreshSignals()) {
            BaseStatusSignal.refreshAll(signals);
        }

        Pose2d robotPose = RobotState.getInstance()
                .getRobotPose()
                .toPose2d(); // Get the current robot pose from the state of the whole robot

        if (ZoneConstants.isInLowZone(robotPose.getTranslation())) {
            System.out.println("Down!");
            this.setState(HopperSubsystem.State.DOWN, Priority.CRITICAL);
        }
        final double targetHeight =
                switch (this.targetState) {
                    case UP -> lidUpHeight.get();
                    case DOWN -> lidDownHeight.get();
                };

        // Calculate control effort
        final double leftPidOutput = leftLifterController.calculate(getLeftLidHeight(), targetHeight);
        final double leftOutput = leftPidOutput + leftLifterFF.calculate(leftPidOutput);
        if (!leftLifterController.atSetpoint()) {
            hopperMotorLeft.setVoltage(leftOutput);
        } else {
            hopperMotorLeft.setVoltage(0);
        }

        final double rightPidOutput = rightLifterController.calculate(getRightLidHeight(), targetHeight);
        final double rightOutput = rightPidOutput + rightLifterFF.calculate(rightPidOutput);
        if (!rightLifterController.atSetpoint()) {
            hopperMotorRight.setVoltage(rightOutput);
        } else {
            hopperMotorRight.setVoltage(0);
        }

        if (PeriodicTasks.getInstance().shouldLog()) {
            // Logging
            Logger.recordOutput("HopperSubsystem/TargetHeight", targetHeight);
            Logger.recordOutput("HopperSubsystem/Left/CurrentHeight", getLeftLidHeight());
            Logger.recordOutput("HopperSubsystem/Left/Output", leftOutput);
            Logger.recordOutput("HopperSubsystem/Left/RawMotorPosition", leftLifterPositionSignal.getValueAsDouble());

            Logger.recordOutput("HopperSubsystem/Right/CurrentHeight", getRightLidHeight());
            Logger.recordOutput("HopperSubsystem/Right/Output", rightOutput);
            Logger.recordOutput("HopperSubsystem/Right/RawMotorPosition", rightLifterPositionSignal.getValueAsDouble());
            // current
            Logger.recordOutput("Debug/Current/Hopper/Left", leftLifterCurrentSignal.getValueAsDouble());
            Logger.recordOutput("Debug/Current/Hopper/Right", rightLifterCurrentSignal.getValueAsDouble());
        }
    }

    /** @return The current lid height in meters. */
    private double getLeftLidHeight() {
        return leftLifterPositionSignal.getValueAsDouble() * HopperConstants.HOPPER_LEFT_ROTATIONS_TO_METERS;
    }

    private double getRightLidHeight() {
        return rightLifterPositionSignal.getValueAsDouble() * HopperConstants.HOPPER_RIGHT_ROTATIONS_TO_METERS;
    }
}
