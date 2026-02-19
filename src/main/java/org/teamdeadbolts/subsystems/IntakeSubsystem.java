/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.IntakeConstants;
import org.teamdeadbolts.utils.tuning.ConfigManager;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase {
    public enum State {
        STOWED,
        INTAKE,
        DEPLOYED,
        SHOOT,
    }

    @AutoLogOutput
    private State targetState = State.STOWED;

    private TalonFX armMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_CAN_ID);
    private TalonFX wheelMotor = new TalonFX(IntakeConstants.INTAKE_WHEEL_MOTOR_CAN_ID);
    private CANcoder absEncoder = new CANcoder(IntakeConstants.INTAKE_ABS_ENCODER_CAN_ID);

    private PIDController armController = new PIDController(0.0, 0.0, 0.0);

    private final SavedLoggedNetworkNumber intakeDeployedAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeDeployedAngle", 90.0);
    private final SavedLoggedNetworkNumber intakeStowedAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeStowedAngle", 0.0);
    private final SavedLoggedNetworkNumber intakeShootArmSpeed =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeShootArmSpeed", 5.0); // Degrees per second

    private final SavedLoggedNetworkNumber armControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/kP", 0.1);
    private final SavedLoggedNetworkNumber armControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/kI", 0.0);
    private final SavedLoggedNetworkNumber armControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/kD", 0.0);

    private final SavedLoggedNetworkNumber wheelIntakeVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Intake/WheelIntakeVoltage", 6.0);

    public IntakeSubsystem() {
        ConfigManager.getInstance().onReady(this::reconfigure);
    }

    public void reconfigure() {
        armController.setPID(armControllerP.get(), armControllerI.get(), armControllerD.get());
        IntakeConstants.init();
        armMotor.getConfigurator().apply(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(IntakeConstants.INTAKE_WHEEL_MOTOR_CONFIG);
        absEncoder.getConfigurator().apply(IntakeConstants.INTAKE_ABS_ENCODER_CONFIG);
    }

    public void setState(State newState) {
        targetState = newState;
    }

    public State getState() {
        return targetState;
    }

    @Override
    public void periodic() {
        double currentAngle = absEncoder.getPosition().getValueAsDouble();
        double targetAngle = currentAngle;
        switch (targetState) {
            case STOWED:
                targetAngle = intakeStowedAngle.get();
                wheelMotor.setVoltage(0);
                break;
            case INTAKE:
                targetAngle = intakeDeployedAngle.get();
                wheelMotor.setVoltage(wheelIntakeVoltage.get());
                break;
            case DEPLOYED:
                targetAngle = intakeDeployedAngle.get();
                wheelMotor.setVoltage(0);
                break;
            case SHOOT:
                if (targetAngle < intakeStowedAngle.get() + 1.0) { // Close enough to stowed
                    targetState = State.STOWED;
                    break;
                }
                targetAngle = currentAngle - intakeShootArmSpeed.get() * (1.0 / 50.0);
                wheelMotor.setVoltage(0);

                break;
        }

        double output = armController.calculate(currentAngle, targetAngle);
        armMotor.setVoltage(output);

        Logger.recordOutput("IntakeSubsystem/CurrentAngle", currentAngle);
        Logger.recordOutput("IntakeSubsystem/TargetAngle", targetAngle);
        Logger.recordOutput("IntakeSubsystem/PIDOutput", output);
    }
}
