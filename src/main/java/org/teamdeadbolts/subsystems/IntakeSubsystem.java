/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.PeriodicTasks;
import org.teamdeadbolts.constants.IntakeConstants;
import org.teamdeadbolts.subsystems.logstructs.IntakeData;
import org.teamdeadbolts.utils.MathUtils;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

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
        HALF_HOLD,
        DEPLOYED,
        SHOOT,
        TEST,
        OFF,
    }

    private final CANBus canBus = new CANBus("*");
    private final TalonFX armMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_CAN_ID, canBus);
    private final TalonFX wheelMotor = new TalonFX(IntakeConstants.INTAKE_DRIVE_MOTOR_CAN_ID, canBus);
    private final CANcoder absEncoder = new CANcoder(IntakeConstants.INTAKE_ABS_ENCODER_CAN_ID, canBus);

    private final StatusSignal<Current> armMotorCurrentSignal = armMotor.getSupplyCurrent();
    private final StatusSignal<Current> wheelMotorCurrentSignal = wheelMotor.getSupplyCurrent();
    private final StatusSignal<Angle> absEncoderAngleSignal = absEncoder.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> absEncoderVelocitySignal = absEncoder.getVelocity();
    private final StatusSignal<Voltage> armMotorVoltageSignal = armMotor.getSupplyVoltage();
    private final StatusSignal<Voltage> wheelMotorVoltageSignal = wheelMotor.getSupplyVoltage();

    private final List<BaseStatusSignal> signals = List.of(
            armMotorCurrentSignal,
            wheelMotorCurrentSignal,
            absEncoderAngleSignal,
            absEncoderVelocitySignal,
            armMotorVoltageSignal,
            wheelMotorVoltageSignal);

    private final ProfiledPIDController armController =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
    /* --- Configuration and Tuning --- */
    private final SavedTunableNumber intakeDeployedAngle =
            SavedTunableNumber.get("Tuning/Intake/IntakeDeployedAngle", 90.0);
    private final SavedTunableNumber intakeStowedAngle = SavedTunableNumber.get("Tuning/Intake/IntakeStowedAngle", 0.0);

    private final SavedTunableNumber armControllerP = SavedTunableNumber.get("Tuning/Intake/ArmController/kP", 0.1);
    private final SavedTunableNumber armControllerI = SavedTunableNumber.get("Tuning/Intake/ArmController/kI", 0.0);
    private final SavedTunableNumber armControllerIZone =
            SavedTunableNumber.get("Tuning/Intake/ArmController/IZone/IZone", 20.0); // Deggress
    private final SavedTunableNumber armControllerIMax =
            SavedTunableNumber.get("Tuning/Intake/ArmController/IZone/IMax", 0.0); // Volts

    private final SavedTunableNumber armControllerMaxVel =
            SavedTunableNumber.get("Tuning/Intake/ArmController/MaxVel", 0.0);
    private final SavedTunableNumber armControllerMaxAccel =
            SavedTunableNumber.get("Tuning/Intake/ArmController/MaxAccel", 0.0);
    private final SavedTunableNumber armControllerTol =
            SavedTunableNumber.get("Tuning/Intake/ArmController/TolDeg", 10);

    private final SavedTunableNumber armDropTol = SavedTunableNumber.get("Tuning/Intake/ArmDropTol", 45.0);

    private final SavedTunableNumber wheelIntakeVoltage =
            SavedTunableNumber.get("Tuning/Intake/WheelIntakeVoltage", 6.0);
    private final SavedTunableNumber wheelShootVoltage =
            SavedTunableNumber.get("Tuning/Intake/Shoot/WheelVoltage", 0.0);
    private final SavedTunableNumber intakeWheelStartTol =
            SavedTunableNumber.get("Tuning/Intake/Wheel/StartWheelTol", 30);

    private final SavedTunableNumber armOffsetDeg = SavedTunableNumber.get("Tuning/Intake/ArmOffsetDeg", 0);

    private final SavedTunableNumber armFeedforwardKs = SavedTunableNumber.get("Tuning/Intake/ArmFeedforward/kS", 0);
    private final SavedTunableNumber armFeedforwardKg = SavedTunableNumber.get("Tuning/Intake/ArmFeedforward/kG", 0);

    private final SavedTunableNumber observerGain = SavedTunableNumber.get("Tuning/Intake/Observer/Gain", 0.0);
    private final SavedTunableNumber maxObserverVolts = SavedTunableNumber.get("Tuning/Intake/Observer/MaxVolts", 0.0);

    private final SavedTunableNumber intakeShootMinAngle = SavedTunableNumber.get("Tuning/Intake/Shoot/MinAngle", 0);
    private final SavedTunableNumber intakeShootMaxAngle = SavedTunableNumber.get("Tuning/Intake/Shoot/MaxAngle", 0);
    private final SavedTunableNumber intakeShootFreq = SavedTunableNumber.get("Tuning/Intake/Shoot/Freq", 0);

    private final SavedTunableNumber halfHoldAngle = SavedTunableNumber.get("Tuning/Intake/HalfHoldAngle", 0);
    private final SavedTunableNumber intakeTestAngle = SavedTunableNumber.get("Tuning/Intake/TestAngle", 0);

    private double currentAngle = 0;
    private double disturbanceAccumulator = 0.0;

    public IntakeSubsystem() {
        super(State.OFF);
        this.currentAngle = MathUtil.inputModulus(
                Units.rotationsToRadians(absEncoderAngleSignal.getValueAsDouble())
                        - Units.degreesToRadians(armOffsetDeg.get()),
                0,
                Math.PI * 2);
        this.targetState = MathUtils.inRange(
                        Math.abs(currentAngle - Units.degreesToRadians(intakeStowedAngle.get())), 0, Math.PI / 4)
                ? State.STOWED
                : State.DEPLOYED; // TODO: A little janky but ok

        armController.enableContinuousInput(-Math.PI, Math.PI);

        // Register refreshables for real-time tuning
        armControllerP.addRefreshable(this);
        armControllerI.addRefreshable(this);
        armControllerTol.addRefreshable(this);
        armControllerIZone.addRefreshable(this);
        armControllerIMax.addRefreshable(this);
        armControllerMaxVel.addRefreshable(this);
        armControllerMaxAccel.addRefreshable(this);
        wheelIntakeVoltage.addRefreshable(this);
        armFeedforwardKg.addRefreshable(this);
        armFeedforwardKs.addRefreshable(this);
    }

    @Override
    public void refresh() {
        armController.setPID(armControllerP.get(), armControllerI.get(), 0.0);
        armController.setConstraints(new TrapezoidProfile.Constraints(
                Units.degreesToRadians(armControllerMaxVel.get()),
                Units.degreesToRadians(armControllerMaxAccel.get())));
        armController.setTolerance(Units.degreesToRadians(armControllerTol.get()));

        armController.setIZone(armControllerIZone.get());
        armController.setIntegratorRange(0, armControllerIMax.get());

        armFeedforward.setKs(armFeedforwardKs.get());
        armFeedforward.setKg(armFeedforwardKg.get());

        IntakeConstants.init();
        armMotor.getConfigurator().apply(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);
        wheelMotor.getConfigurator().apply(IntakeConstants.INTAKE_WHEEL_MOTOR_CONFIG);
        absEncoder.getConfigurator().apply(IntakeConstants.INTAKE_ABS_ENCODER_CONFIG);
    }

    @Override
    protected void onStateChange(final State to, final State from) {
        armController.reset(new TrapezoidProfile.State(
                currentAngle, Units.rotationsToRadians(absEncoderVelocitySignal.getValueAsDouble())));

        disturbanceAccumulator = 0.0;
    }

    public List<BaseStatusSignal> getSignals() {
        return signals;
    }

    @Override
    public void subsystemPeriodic() {
        if (PeriodicTasks.getInstance().shouldRefreshSignals()) {
            BaseStatusSignal.refreshAll(signals);
        }

        currentAngle = MathUtil.inputModulus(
                Units.rotationsToRadians(absEncoderAngleSignal.getValueAsDouble())
                        - Units.degreesToRadians(armOffsetDeg.get()),
                -Math.PI,
                Math.PI);

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
                if (Math.abs(MathUtils.getShortestDistance(currentAngle, targetAngle.get()))
                        <= Units.degreesToRadians(intakeWheelStartTol.get())) {
                    wheelMotor.setVoltage(wheelIntakeVoltage.get());
                } else {
                    wheelMotor.setVoltage(0);
                }
            }
            case OUTTAKE -> {
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(-wheelIntakeVoltage.get());
            }
            case DEPLOYED -> {
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(0);
            }
            case HALF_HOLD -> {
                targetAngle = Optional.of(Units.degreesToRadians(halfHoldAngle.get()));
                wheelMotor.setVoltage(0.0);
            }
            case SHOOT -> {
                final double time = Timer.getFPGATimestamp();
                final double freq = intakeShootFreq.get();
                final double minAngle = Units.degreesToRadians(intakeShootMinAngle.get());
                final double maxAngle = Units.degreesToRadians(intakeShootMaxAngle.get());

                final double midAngle = (maxAngle + minAngle) / 2.0;
                final double amplitude = (maxAngle - minAngle) / 2.0;
                final double target = midAngle + amplitude * Math.sin(2 * Math.PI * freq * time);

                targetAngle = Optional.of(target);
                wheelMotor.setVoltage(wheelShootVoltage.get());
            }
            case TEST -> {
                targetAngle = Optional.of(Units.degreesToRadians(intakeTestAngle.get()));
                wheelMotor.setVoltage(0.0);
            }
        }

        IntakeData intakeData = new IntakeData();

        if (targetAngle.isPresent()) {
            final double pidVolts = armController.calculate(currentAngle, targetAngle.get());
            final TrapezoidProfile.State setpoint = armController.getSetpoint();

            final double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
            final double actualVelocity =
                    Units.rotationsToRadians(absEncoder.getVelocity().getValueAsDouble());
            final double vError = setpoint.velocity - actualVelocity;

            disturbanceAccumulator += vError * observerGain.get();
            disturbanceAccumulator =
                    MathUtil.clamp(disturbanceAccumulator, -maxObserverVolts.get(), maxObserverVolts.get());
            final double totalVolts = feedforward + pidVolts + disturbanceAccumulator;
            final boolean shouldDrop = Math.abs(MathUtils.getShortestDistance(currentAngle, targetAngle.get()))
                            <= Units.degreesToRadians(armDropTol.get())
                    && (targetState == State.INTAKE || targetState == State.DEPLOYED);
            if (shouldDrop) {
                armMotor.setVoltage(0);
            } else {
                armMotor.setVoltage(totalVolts);
            }

            //            Logger.recordOutput("IntakeSubsystem/Arm/AtGoal", armController.atGoal());
            //            Logger.recordOutput("IntakeSubsystem/Arm/SetpointPos",
            // Units.radiansToDegrees(setpoint.position));
            //            Logger.recordOutput("IntakeSubsystem/Arm/SetpointVel",
            // Units.radiansToDegrees(setpoint.velocity));
            //            Logger.recordOutput(
            //                    "IntakeSubsystem/Arm/PidPosError",
            // Units.radiansToDegrees(armController.getPositionError()));
            //            Logger.recordOutput("IntakeSubsystem/Arm/CurrVelocity",
            // Units.radiansToDegrees(actualVelocity));
            //            Logger.recordOutput("IntakeSubsystem/Arm/TargetAngle",
            // Units.radiansToDegrees(targetAngle.get()));
            //            final TrapezoidProfile.State state = armController.getSetpoint();
            //            Logger.recordOutput("IntakeSubsystem/Arm/Trapizoid/SetpointPos",
            // Units.radiansToDegrees(state.position));
            //            Logger.recordOutput("IntakeSubsystem/Arm/Trapizoid/SetpointVel",
            // Units.radiansToDegrees(state.velocity));
            //
            //            // Log control effort breakdown
            //            Logger.recordOutput("IntakeSubsystem/Arm/Effort/PIDVolts", pidVolts);
            //            Logger.recordOutput("IntakeSubsystem/Arm/Effort/FFVolts", feedforward);
            //            Logger.recordOutput("IntakeSubsystem/Arm/Effort/ObserverVolts", disturbanceAccumulator);

            intakeData.armAtGoal = armController.atGoal();
            intakeData.armSetpointPos = Units.radiansToDegrees(setpoint.position);
            intakeData.armSetpointVel = Units.radiansToDegrees(setpoint.velocity);
            intakeData.armPidPosError = Units.radiansToDegrees(armController.getPositionError());
            intakeData.armCurrVelocity = Units.radiansToDegrees(actualVelocity);
            intakeData.armTargetAngle = Units.radiansToDegrees(targetAngle.get());

            final TrapezoidProfile.State state = armController.getSetpoint();
            intakeData.armTrapizoidSetpointPos = Units.radiansToDegrees(state.position);
            intakeData.armTrapizoidSetpointVel = Units.radiansToDegrees(state.velocity);

            // Log control effort breakdown
            intakeData.armEffortPIDVolts = pidVolts;
            intakeData.armEffortFFVolts = feedforward;
            intakeData.armEffortObserverVolts = disturbanceAccumulator;
        }

        if (PeriodicTasks.getInstance().shouldLog()) {
            Logger.recordOutput("IntakeSubsystem/TargetState", targetState);
        }

        //        Logger.recordOutput("IntakeSubsystem/Arm/CurrentAngle", Units.radiansToDegrees(currentAngle));
        //        Logger.recordOutput("IntakeSubsystem/Arm/OutputVolts", armMotorVoltageSignal.getValueAsDouble());
        //
        //        Logger.recordOutput("IntakeSubsystem/Wheels/Voltage", wheelMotorVoltageSignal.getValueAsDouble());

        intakeData.armCurrentAngle = Units.radiansToDegrees(currentAngle);
        intakeData.armOutputVolts = armMotorVoltageSignal.getValueAsDouble();
        intakeData.wheelsVoltage = wheelMotorVoltageSignal.getValueAsDouble();

        if (PeriodicTasks.getInstance().shouldLog()) {
            Logger.recordOutput("IntakeSubsystem", intakeData);

            // Current monitoring
            Logger.recordOutput("Debug/Current/Intake/Arm", armMotorCurrentSignal.getValueAsDouble());
            Logger.recordOutput("Debug/Current/Intake/Wheel", wheelMotorCurrentSignal.getValueAsDouble());
        }
    }

    public boolean armAtGoal() {
        return armController.atGoal();
    }
}
