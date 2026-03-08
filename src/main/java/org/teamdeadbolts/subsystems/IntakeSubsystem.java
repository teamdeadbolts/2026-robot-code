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
import edu.wpi.first.wpilibj.Timer;
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
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
    /* --- Configuration and Tuning --- */
    private final SavedLoggedNetworkNumber intakeDeployedAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeDeployedAngle", 90.0);
    private final SavedLoggedNetworkNumber intakeStowedAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeStowedAngle", 0.0);

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

    private final SavedLoggedNetworkNumber wheelIntakeVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Intake/WheelIntakeVoltage", 6.0);
    private final SavedLoggedNetworkNumber wheelShootVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Intake/Shoot/WheelVoltage", 0.0);

    private final SavedLoggedNetworkNumber armOffsetDeg = SavedLoggedNetworkNumber.get("Tuning/Intake/ArmOffsetDeg", 0);

    private final SavedLoggedNetworkNumber armFeedforwardKs =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kS", 0);
    private final SavedLoggedNetworkNumber armFeedforwardKg =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kG", 0);

    private final SavedLoggedNetworkNumber observerGain =
            SavedLoggedNetworkNumber.get("Tuning/Intake/Observer/Gain", 0.0);
    private final SavedLoggedNetworkNumber maxObserverVolts =
            SavedLoggedNetworkNumber.get("Tuning/Intake/Observer/MaxVolts", 0.0);

    private final SavedLoggedNetworkNumber intakeShootMinAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/Shoot/MinAngle", 0);
    private final SavedLoggedNetworkNumber intakeShootMaxAngle =
            SavedLoggedNetworkNumber.get("Tuning/Intake/Shoot/MaxAngle", 0);
    private final SavedLoggedNetworkNumber intakeShootFreq =
            SavedLoggedNetworkNumber.get("Tuning/Intake/Shoot/Freq", 0);

    private double currentAngle = 0;
    private double disturbanceAccumulator = 0.0;

    public IntakeSubsystem() {
        this.targetState = State.OFF;
        armController.enableContinuousInput(0, Math.PI * 2);

        // Register refreshables for real-time tuning
        armControllerP.addRefreshable(this);
        armControllerI.addRefreshable(this);
        armControllerD.addRefreshable(this);
        armControllerMaxVel.addRefreshable(this);
        armControllerMaxAccel.addRefreshable(this);
        wheelIntakeVoltage.addRefreshable(this);
        armFeedforwardKg.addRefreshable(this);
        armFeedforwardKs.addRefreshable(this);
    }

    @Override
    public void refresh() {
        armController.setPID(armControllerP.get(), armControllerI.get(), armControllerD.get());
        armController.setConstraints(new TrapezoidProfile.Constraints(
                Units.degreesToRadians(armControllerMaxVel.get()),
                Units.degreesToRadians(armControllerMaxAccel.get())));

        armController.setTolerance(Units.degreesToRadians(armControllerTol.get()));
        armFeedforward.setKs(armFeedforwardKs.get());
        armFeedforward.setKg(armFeedforwardKg.get());

        IntakeConstants.init();
        armMotor.getConfigurator().apply(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(IntakeConstants.INTAKE_WHEEL_MOTOR_CONFIG);
        absEncoder.getConfigurator().apply(IntakeConstants.INTAKE_ABS_ENCODER_CONFIG);
    }

    @Override
    protected void onStateChange(State to, State from) {
        armController.reset(new TrapezoidProfile.State(
                currentAngle, Units.rotationsToRadians(absEncoder.getVelocity().getValueAsDouble())));

        disturbanceAccumulator = 0.0;
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
            case SHOOT -> {
                double time = Timer.getFPGATimestamp();
                double freq = intakeShootFreq.get();
                double minAngle = Units.degreesToRadians(intakeShootMinAngle.get());
                double maxAngle = Units.degreesToRadians(intakeShootMaxAngle.get());

                double midAngle = (maxAngle + minAngle) / 2.0;
                double amplitude = (maxAngle - minAngle) / 2.0;
                double target = midAngle + amplitude * Math.sin(2 * Math.PI * freq * time);

                targetAngle = Optional.of(target);
                wheelMotor.setVoltage(wheelShootVoltage.get());
            }
        }

        if (targetAngle.isPresent()) {
            double pidVolts = armController.calculate(currentAngle, targetAngle.get());
            TrapezoidProfile.State setpoint = armController.getSetpoint();

            double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
            double actualVelocity =
                    Units.rotationsToRadians(absEncoder.getVelocity().getValueAsDouble());
            double vError = setpoint.velocity - actualVelocity;

            disturbanceAccumulator += vError * observerGain.get();
            disturbanceAccumulator =
                    MathUtil.clamp(disturbanceAccumulator, -maxObserverVolts.get(), maxObserverVolts.get());
            double totalVolts = feedforward + pidVolts + disturbanceAccumulator;

            if ((targetState == State.DEPLOYED || targetState == State.STOWED) && armController.atGoal()) {
                armMotor.setVoltage(0);
            } else {
                armMotor.set(totalVolts);
            }
            Logger.recordOutput("IntakeSubsystem/SetpointPos", Units.radiansToDegrees(setpoint.position));
            Logger.recordOutput("IntakeSubsystem/SetpointVel", Units.radiansToDegrees(setpoint.velocity));
            Logger.recordOutput("IntakeSubsystem/CurrVelocity", Units.radiansToDegrees(actualVelocity));
            Logger.recordOutput("IntakeSubsystem/TargetAngle", Units.radiansToDegrees(targetAngle.get()));

            // Log control effort breakdown
            Logger.recordOutput("IntakeSubsystem/Effort/PIDVolts", pidVolts);
            Logger.recordOutput("IntakeSubsystem/Effort/FFVolts", feedforward);
            Logger.recordOutput("IntakeSubsystem/Effort/ObserverVolts", disturbanceAccumulator);
        }

        Logger.recordOutput("IntakeSubsystem/CurrentAngle", Units.radiansToDegrees(currentAngle));
        Logger.recordOutput("IntakeSubsystem/TargetState", targetState);
        Logger.recordOutput(
                "IntakeSubsystem/OutputVolts", armMotor.getMotorVoltage().getValueAsDouble());

        // Current monitoring
        Logger.recordOutput(
                "Debug/Current/Intake/Arm", armMotor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(
                "Debug/Current/Intake/Wheel", wheelMotor.getSupplyCurrent().getValueAsDouble());
    }
}
