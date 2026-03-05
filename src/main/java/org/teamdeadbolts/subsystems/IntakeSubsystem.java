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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.IntakeConstants;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase implements Refreshable {
    public enum State {
        STOWED,
        INTAKE,
        OUTTAKE,
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

    private final SavedLoggedNetworkNumber armControllerMaxVel =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/MaxVel", 0.0); // Degrees per second
    private final SavedLoggedNetworkNumber armControllerMaxAccel =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/MaxAccel", 0.0); // Degrees per second squared

    private final SavedLoggedNetworkNumber armControllerTol =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/TolDeg", 10);

    private final SavedLoggedNetworkNumber armCutoffVelTol =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmController/VelTolDeg", 10);

    private final SavedLoggedNetworkNumber armFeedforwardKs =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kS", 0.0);
    private final SavedLoggedNetworkNumber armFeedforwardKg =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kG", 0.0);
    private final SavedLoggedNetworkNumber armFeedforwardKv =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ArmFeedforward/kV", 0.0);

    private final SavedLoggedNetworkNumber wheelIntakeVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Intake/WheelIntakeVoltage", 6.0);

    private final SavedLoggedNetworkNumber wheelSlowIntakeVoltage =
            SavedLoggedNetworkNumber.get("Tuning/Intake/WheelSlowIntakeVoltage", 1.0);

    private final SavedLoggedNetworkNumber jiggleFrequency =
            SavedLoggedNetworkNumber.get("Tuning/Intake/JiggleFrequency", 0.25);
    private final SavedLoggedNetworkNumber intakeJiggleVolts =
            SavedLoggedNetworkNumber.get("Tuning/Intake/IntakeJiggleVolts", 4.0);

    private final SavedLoggedNetworkNumber shootJiggleTolerance =
            SavedLoggedNetworkNumber.get("Tuning/Intake/ShootJiggleTolerance", 5);

    private final SavedLoggedNetworkNumber armOffsetDeg = SavedLoggedNetworkNumber.get("Tuning/Intake/ArmOffsetDeg", 0);

    private double shootTargetAngle = 0;

    public IntakeSubsystem() {
        armController.enableContinuousInput(0, Math.PI * 2);
        armControllerP.addRefreshable(this);
        armControllerI.addRefreshable(this);
        armControllerD.addRefreshable(this);
        armControllerMaxVel.addRefreshable(this);
        armControllerMaxAccel.addRefreshable(this);
        armFeedforwardKg.addRefreshable(this);
        armFeedforwardKv.addRefreshable(this);
        wheelIntakeVoltage.addRefreshable(this);
        //        refresh();
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
        double currentAngle = MathUtil.inputModulus(
                Units.rotationsToRadians(absEncoder.getPosition().getValueAsDouble())
                        - Units.degreesToRadians(armOffsetDeg.get()),
                0,
                Math.PI * 2);
        Optional<Double> targetAngle = Optional.empty();
        switch (targetState) {
            case OFF:
                wheelMotor.setVoltage(0);
                armMotor.setVoltage(0);
                break;
            case STOWED:
                shootTargetAngle = 0;
                targetAngle = Optional.of(Units.degreesToRadians(intakeStowedAngle.get()));
                wheelMotor.setVoltage(0);
                break;
            case INTAKE:
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(wheelIntakeVoltage.get());
                break;
            case OUTTAKE:
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(-wheelIntakeVoltage.get());
                break;
            case DEPLOYED:
                targetAngle = Optional.of(Units.degreesToRadians(intakeDeployedAngle.get()));
                wheelMotor.setVoltage(0);
                break;
            case SHOOT:
                double jiggleVolts =
                        Math.sin(2 * Math.PI * jiggleFrequency.get() * (System.currentTimeMillis() / 1000.0))
                                * intakeJiggleVolts.get();
                if (Math.abs(currentAngle - intakeDeployedAngle.get())
                                <= Units.degreesToRadians(shootJiggleTolerance.get())
                        && jiggleVolts > 0) {
                    armMotor.setVoltage(jiggleVolts);
                } else {
                    armMotor.setVoltage(0);
                }
                Logger.recordOutput("IntakeSubsystem/JiggleVolts", jiggleVolts);
                wheelMotor.setVoltage(wheelSlowIntakeVoltage.get());

                break;
        }

        if (targetAngle.isPresent()) {
            TrapezoidProfile.State setpoint = armController.getSetpoint();
            double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
            double pidOut = armController.calculate(currentAngle, targetAngle.get());

            if ((targetState == State.DEPLOYED || targetState == State.STOWED)
                    && Math.abs(targetAngle.get() - currentAngle) < Units.degreesToRadians(armControllerTol.get())
                    && absEncoder.getVelocity().getValueAsDouble() < Units.degreesToRotations(armCutoffVelTol.get())) {
                armMotor.setVoltage(0);
            } else if (targetState != State.SHOOT) {
                armMotor.setVoltage(feedforward + pidOut);
            }

            Logger.recordOutput("IntakeSubsystem/SetpointPos", Units.radiansToDegrees(setpoint.position));
            Logger.recordOutput("IntakeSubsystem/SetpointVel", Units.radiansToDegrees(setpoint.velocity));
            Logger.recordOutput(
                    "IntakeSubsystem/CurrVelocity",
                    Units.rotationsToDegrees(absEncoder.getVelocity().getValueAsDouble()));

            Logger.recordOutput("IntakeSubsystem/TargetAngle", Units.radiansToDegrees(targetAngle.get()));
            Logger.recordOutput("IntakeSubsystem/Output", feedforward + pidOut);
            Logger.recordOutput("IntakeSubsystem/FeedforwardOut", feedforward);
            Logger.recordOutput("IntakeSubsystem/PidOut", pidOut);
            Logger.recordOutput("IntakeSubsystem/kG", armFeedforward.getKg());
            Logger.recordOutput("IntakeSubsystem/kGT", armFeedforwardKg.get());
        }

        Logger.recordOutput("IntakeSubsystem/CurrentAngle", Units.radiansToDegrees(currentAngle));
        Logger.recordOutput("IntakeSubsystem/TargetState", targetState);
        Logger.recordOutput(
                "IntakeSubsystem/OutputVolts", armMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(
                "IntakeSubsystem/OutputCurrent", armMotor.getStatorCurrent().getValueAsDouble());
    }
}
