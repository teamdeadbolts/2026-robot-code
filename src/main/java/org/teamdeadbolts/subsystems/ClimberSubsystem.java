/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.ClimberConstants;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ClimberSubsystem extends SubsystemBase implements Refreshable {
    public enum State {
        STOWED,
        CLIMB_READY,
        CLIMB;
    }

    private CANBus canBus = new CANBus("*");
    private final TalonFX climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID, canBus);

    private final SavedLoggedNetworkNumber climberControllerKp =
            SavedLoggedNetworkNumber.get("Tuning/Climber/ClimberController/Kp", 0.0, this);

    private final SavedLoggedNetworkNumber climberControllerKi =
            SavedLoggedNetworkNumber.get("Tuning/Climber/ClimberController/Ki", 0.0, this);

    private final SavedLoggedNetworkNumber climberControllerKd =
            SavedLoggedNetworkNumber.get("Tuning/Climber/ClimberController/Kd", 0.0, this);

    private final SavedLoggedNetworkNumber climberStowedPosition =
            SavedLoggedNetworkNumber.get("Tuning/Climber/ClimberStowedPosition", 0.0);

    private final SavedLoggedNetworkNumber climberClimbReadyPosition =
            SavedLoggedNetworkNumber.get("Tuning/Climber/ClimberClimbReadyPosition", 0.0);

    private final SavedLoggedNetworkNumber climberClimbPosition =
            SavedLoggedNetworkNumber.get("Tuning/Climber/ClimberClimbPosition", 0.0);

    private PIDController climberController = new PIDController(0, 0, 0);

    private State state = State.STOWED;

    public ClimberSubsystem() {
        ConfigManager.getInstance().onReady(this::refresh);
    }

    @Override
    public void refresh() {
        ClimberConstants.init();
        climberMotor.getConfigurator().apply(ClimberConstants.CLIMBER_MOTOR_CONFIG);
        climberController.setP(climberControllerKp.get());
        climberController.setI(climberControllerKi.get());
        climberController.setD(climberControllerKd.get());
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        switch (this.state) {
            case CLIMB:
                climberController.setSetpoint(climberClimbPosition.get());
                break;
            case CLIMB_READY:
                climberController.setSetpoint(climberClimbReadyPosition.get());
                break;
            case STOWED:
                climberController.setSetpoint(climberStowedPosition.get());
                break;
        }

        double currentPosition = climberMotor.getPosition().getValueAsDouble(); // TODO: roations to meters??
        double output = climberController.calculate(currentPosition);

        climberMotor.setVoltage(output);

        Logger.recordOutput("Climber/CurrentPosition", currentPosition);
        Logger.recordOutput("Climber/Output", output);
        Logger.recordOutput("Climber/Setpoint", climberController.getSetpoint());
        Logger.recordOutput("Climber/State", state);
    }
}
