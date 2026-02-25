/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
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
        OFF,
    }

    // @AutoLogOutput
    private State targetState = State.OFF;

    private CANBus canBus = new CANBus("*");
    private TalonFX armMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_CAN_ID, canBus);
    private TalonFX wheelMotor = new TalonFX(IntakeConstants.INTAKE_DRIVE_MOTOR_CAN_ID, canBus);
    private CANcoder absEncoder = new CANcoder(IntakeConstants.INTAKE_ABS_ENCODER_CAN_ID, canBus);

    private ProfiledPIDController armController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

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

    private SavedLoggedNetworkNumber armControllerMaxVel =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/MaxVel", 0.0); // Degrees per second
    private SavedLoggedNetworkNumber armControllerMaxAccel =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/MaxAccel", 0.0); // Degrees per second squared

    private SavedLoggedNetworkNumber armFeedforwardKs =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kS", 0.0);
    private SavedLoggedNetworkNumber armFeedforwardKg =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kG", 0.0);
    private SavedLoggedNetworkNumber armFeedforwardKv =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kV", 0.0);

    private SavedLoggedNetworkNumber wheelIntakeVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Intake/WheelIntakeVoltage", 6.0);

    private SavedLoggedNetworkNumber armOffsetDeg = SavedLoggedNetworkNumber.get("Tuning/Intake/ArmOffsetDeg", 0);

    public IntakeSubsystem() {
        ConfigManager.getInstance().onReady(this::reconfigure);
        armControllerP.onChange(armController::setP);
        armControllerI.onChange(armController::setI);
        armControllerD.onChange(armController::setD);
        armControllerMaxVel.onChange(v -> armController.setConstraints(
                new TrapezoidProfile.Constraints(v, Units.degreesToRadians(armControllerMaxAccel.get()))));
        armControllerMaxAccel.onChange(a -> armController.setConstraints(
                new TrapezoidProfile.Constraints(armControllerMaxVel.get(), Units.degreesToRadians(a))));
        armFeedforwardKg.onChange(armFeedforward::setKg);
        armFeedforwardKs.onChange(armFeedforward::setKs);
        armFeedforwardKv.onChange(armFeedforward::setKv);

        // reconfigure();
    }

    public void reconfigure() {
        armController.setPID(armControllerP.get(), armControllerI.get(), armControllerD.get());
        armController.setConstraints(new TrapezoidProfile.Constraints(
                Units.degreesToRadians(armControllerMaxVel.get()),
                Units.degreesToRadians(armControllerMaxAccel.get())));
        armFeedforward.setKg(armFeedforwardKg.get());
        armFeedforward.setKs(armFeedforwardKs.get());
        armFeedforward.setKv(armFeedforwardKv.get());
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
        double currentAngle = Units.rotationsToRadians(absEncoder.getPosition().getValueAsDouble())
                - Units.degreesToRadians(armOffsetDeg.get());
        Optional<Double> targetAngle = Optional.empty();
        switch (targetState) {
            case OFF:
                wheelMotor.setVoltage(0);
                armMotor.setVoltage(0);
                break;
            case STOWED:
                targetAngle = Optional.of(Units.degreesToRadians(intakeStowedAngle.get()));
                wheelMotor.setVoltage(0);
                break;
            case INTAKE:
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(wheelIntakeVoltage.get());
                break;
            case DEPLOYED:
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(0);
                break;
            case SHOOT:
                targetAngle = Optional.of(currentAngle - intakeShootArmSpeed.get() * (1.0 / 50.0));
                if (targetAngle.isPresent()
                        && targetAngle.get()
                                < Units.degreesToRadians(intakeStowedAngle.get() + 1.0)) { // Close enough to stowed
                    targetState = State.STOWED;
                    break;
                }
                wheelMotor.setVoltage(0);

                break;
        }

        if (targetAngle.isPresent()) {
            TrapezoidProfile.State setpoint = armController.getSetpoint();
            double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
            double pidOut = armController.calculate(currentAngle, targetAngle.get());

            armMotor.setVoltage(feedforward + pidOut);

            Logger.recordOutput("IntakeSubsystem/TargetAngle", Units.radiansToDegrees(targetAngle.get()));
            Logger.recordOutput("IntakeSubsystem/Output", feedforward + pidOut);
            Logger.recordOutput("IntakeSubsystem/FeedforwardOut", feedforward);
            Logger.recordOutput("IntakeSubsystem/PidOut", pidOut);
            Logger.recordOutput("IntakeSubsystem/kG", armFeedforward.getKg());
            Logger.recordOutput("IntakeSubsystem/kGT", armFeedforwardKg.get());
        }

        Logger.recordOutput("IntakeSubsystem/CurrentAngle", Units.radiansToDegrees(currentAngle));
        Logger.recordOutput("IntakeSubsystem/TargetState", targetState);
    }
}
