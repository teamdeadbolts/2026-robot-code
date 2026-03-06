/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.IntakeConstants;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Manages the intake assembly, including the rotating arm and intake rollers.
 * Operates as a {@link StatefulSubsystem} using a {@link ProfiledPIDController}
 * for smooth arm motion and {@link ArmFeedforward} for gravity/load compensation.
 */
public class IntakeSubsystem extends StatefulSubsystem<IntakeSubsystem.State> implements Refreshable {
    public enum State {
        STOWED,
        INTAKE,
        OUTTAKE,
        DEPLOYED,
        SHOOT,
        OFF,
    }

    private final CANBus canBus = new CANBus("*");
    private final TalonFX armMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_CAN_ID, canBus);
    private final TalonFX wheelMotor = new TalonFX(IntakeConstants.INTAKE_DRIVE_MOTOR_CAN_ID, canBus);
    private final CANcoder absEncoder = new CANcoder(IntakeConstants.INTAKE_ABS_ENCODER_CAN_ID, canBus);

    private final ProfiledPIDController armController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
    private final ArmFeedforward armShootFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);

    /* --- Configuration and Tuning --- */
    private final SavedLoggedNetworkNumber intakeDeployedAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeDeployedAngle", 90.0);
    private final SavedLoggedNetworkNumber intakeStowedAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeStowedAngle", 0.0);
    private final SavedLoggedNetworkNumber intakeShootArmAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeShootArmAngle", 45.0);

    private final SavedLoggedNetworkNumber armControllerP =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/kP", 0.1);
    private final SavedLoggedNetworkNumber armControllerI =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/kI", 0.0);
    private final SavedLoggedNetworkNumber armControllerD =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/kD", 0.0);
    private final SavedLoggedNetworkNumber armControllerMaxVel =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/MaxVel", 0.0);
    private final SavedLoggedNetworkNumber armControllerMaxAccel =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/MaxAccel", 0.0);
    private final SavedLoggedNetworkNumber armControllerTol =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/TolDeg", 10);
    private final SavedLoggedNetworkNumber armCutoffVelTol =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/VelTolDeg", 10);

    private final SavedLoggedNetworkNumber armFeedforwardKs =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmEmptyFeedforward/kS", 0.0);
    private final SavedLoggedNetworkNumber armFeedforwardKg =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmEmptyFeedforward/kG", 0.0);
    private final SavedLoggedNetworkNumber armFeedforwardKv =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmEmptyFeedforward/kV", 0.0);
    private final SavedLoggedNetworkNumber armShootFeedforwardKs =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmShootFeedforward/kS", 0.0);
    private final SavedLoggedNetworkNumber armShootFeedforwardKg =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmShootFeedforward/kG", 0.0);
    private final SavedLoggedNetworkNumber armShootFeedforwardKv =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmShootFeedforward/kV", 0.0);

    private final SavedLoggedNetworkNumber wheelIntakeVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Intake/WheelIntakeVoltage", 6.0);
    private final SavedLoggedNetworkNumber armOffsetDeg = SavedLoggedNetworkNumber.get("Tuning/Intake/ArmOffsetDeg", 0);

    private double currentAngle = 0;

    public IntakeSubsystem() {
        this.targetState = State.OFF;
        armController.enableContinuousInput(0, Math.PI * 2);

        // Register refreshables for real-time tuning
        armControllerP.addRefreshable(this);
        armControllerI.addRefreshable(this);
        armControllerD.addRefreshable(this);
        armControllerMaxVel.addRefreshable(this);
        armControllerMaxAccel.addRefreshable(this);
        armFeedforwardKg.addRefreshable(this);
        armFeedforwardKv.addRefreshable(this);
        armShootFeedforwardKg.addRefreshable(this);
        armShootFeedforwardKs.addRefreshable(this);
        armShootFeedforwardKv.addRefreshable(this);
        wheelIntakeVoltage.addRefreshable(this);
    }

    @Override
    public void refresh() {
        armController.setPID(armControllerP.get(), armControllerI.get(), armControllerD.get());
        armController.setConstraints(new TrapezoidProfile.Constraints(
                Units.degreesToRadians(armControllerMaxVel.get()),
                Units.degreesToRadians(armControllerMaxAccel.get())));

        armController.setTolerance(Units.degreesToRadians(armControllerTol.get()));
        armFeedforward.setKg(armFeedforwardKg.get());
        armFeedforward.setKs(armFeedforwardKs.get());
        armFeedforward.setKv(armFeedforwardKv.get());
        armShootFeedforward.setKs(armShootFeedforwardKs.get());
        armShootFeedforward.setKg(armShootFeedforwardKg.get());
        armShootFeedforward.setKv(armShootFeedforwardKv.get());

        IntakeConstants.init();
        armMotor.getConfigurator().apply(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(IntakeConstants.INTAKE_WHEEL_MOTOR_CONFIG);
        absEncoder.getConfigurator().apply(IntakeConstants.INTAKE_ABS_ENCODER_CONFIG);
    }

    @Override
    protected void onStateChange(State to, State from) {
        armController.reset(new TrapezoidProfile.State(
                currentAngle, Units.rotationsToRadians(absEncoder.getVelocity().getValueAsDouble())));
    }

    @Override
    public void periodic() {
        currentAngle = MathUtil.inputModulus(
                Units.rotationsToRadians(absEncoder.getPosition().getValueAsDouble())
                        - Units.degreesToRadians(armOffsetDeg.get()),
                0,
                Math.PI * 2);

        Optional<Double> targetAngle = Optional.empty();
        switch (targetState) {
            case OFF -> {
                wheelMotor.setVoltage(0);
                armMotor.setVoltage(0);
            }
            case STOWED -> {
                targetAngle = Optional.of(Units.degreesToRadians(intakeStowedAngle.get()));
                wheelMotor.setVoltage(0);
            }
            case INTAKE -> {
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(wheelIntakeVoltage.get());
            }
            case OUTTAKE -> {
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(-wheelIntakeVoltage.get());
            }
            case DEPLOYED -> {
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(0);
            }
            case SHOOT -> targetAngle = Optional.of(Units.degreesToRadians(intakeShootArmAngle.get()));
        }

        if (targetAngle.isPresent()) {
            TrapezoidProfile.State setpoint = armController.getSetpoint();
            double feedforward = targetState == State.SHOOT
                    ? armShootFeedforward.calculate(setpoint.position, setpoint.velocity)
                    : armFeedforward.calculate(Math.PI / 4.0, 0);
            double pidOut = armController.calculate(currentAngle, targetAngle.get());

            // Soft-stop logic for static positions to save power/reduce oscillation
            boolean isAtTarget =
                    Math.abs(targetAngle.get() - currentAngle) < Units.degreesToRadians(armControllerTol.get());
            boolean isAtVelocity =
                    absEncoder.getVelocity().getValueAsDouble() < Units.degreesToRotations(armCutoffVelTol.get());

            if ((targetState == State.DEPLOYED || targetState == State.STOWED) && isAtTarget && isAtVelocity) {
                armMotor.setVoltage(0);
            } else {
                armMotor.setVoltage(feedforward + pidOut);
            }

            // Telemetry logging
            Logger.recordOutput("IntakeSubsystem/SetpointPos", Units.radiansToDegrees(setpoint.position));
            Logger.recordOutput("IntakeSubsystem/SetpointVel", Units.radiansToDegrees(setpoint.velocity));
            Logger.recordOutput(
                    "IntakeSubsystem/CurrVelocity",
                    Units.rotationsToDegrees(absEncoder.getVelocity().getValueAsDouble()));
            Logger.recordOutput("IntakeSubsystem/TargetAngle", Units.radiansToDegrees(targetAngle.get()));
        }

        Logger.recordOutput("IntakeSubsystem/CurrentAngle", Units.radiansToDegrees(currentAngle));
        Logger.recordOutput("IntakeSubsystem/TargetState", targetState);
        Logger.recordOutput(
                "IntakeSubsystem/OutputVolts", armMotor.getMotorVoltage().getValueAsDouble());
    }
}
