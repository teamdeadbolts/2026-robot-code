/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.IndexerConstants;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Manages the indexing subsystem responsible for handling game pieces
 * between the intake and the shooter. Operates via two motors: a floor
 * motor for transport and a kicker motor for final feeding.
 */
public class IndexerSubsystem extends StatefulSubsystem<IndexerSubsystem.State> implements Refreshable {
    public enum State {
        OFF,
        JIGGLE,
        SHOOT,
        REVERSE,
    }

    private final CANBus canBus = new CANBus("*");
    private final TalonFX floorMotor = new TalonFX(IndexerConstants.INDEXER_FLOOR_MOTOR_CAN_ID, canBus);
    private final TalonFX kickerMotor = new TalonFX(IndexerConstants.INDEXER_KICKER_MOTOR_CAN_ID, canBus);

    /* --- Tuning Parameters --- */
    private final SavedLoggedNetworkNumber floorMotorIntakeVolts =
            SavedLoggedNetworkNumber.get("Tuning/Indexer/IndexerFloorMotorIntakeVolts", 6.0);
    private final SavedLoggedNetworkNumber floorMotorShootVolts =
            SavedLoggedNetworkNumber.get("Tuning/Indexer/IndexerFloorMotorShootVolts", 6.0);
    private final SavedLoggedNetworkNumber floorMotorJiggleVolts =
            SavedLoggedNetworkNumber.get("Tuning/Indexer/IndexerFloorMotorJiggleVolts", 3.0);
    private final SavedLoggedNetworkNumber kickerMotorShootVolts =
            SavedLoggedNetworkNumber.get("Tuning/Indexer/IndexerKickerMotorShootVolts", 6.0);
    private final SavedLoggedNetworkNumber jiggleFrequency =
            SavedLoggedNetworkNumber.get("Tuning/Indexer/IndexerJiggleFrequency", 1.0);

    public IndexerSubsystem() {
        this.targetState = State.OFF;

        // Register tuning parameters for real-time updates
        floorMotorIntakeVolts.addRefreshable(this);
        floorMotorShootVolts.addRefreshable(this);
        floorMotorJiggleVolts.addRefreshable(this);
        kickerMotorShootVolts.addRefreshable(this);
        jiggleFrequency.addRefreshable(this);
    }

    @Override
    public void refresh() {
        IndexerConstants.init();
        floorMotor.getConfigurator().apply(IndexerConstants.INDEXER_FLOOR_MOTOR_CONFIG);
        kickerMotor.getConfigurator().apply(IndexerConstants.INDEXER_KICKER_MOTOR_CONFIG);
    }

    @Override
    protected void onStateChange(State to, State from) {}

    @Override
    public void periodic() {
        switch (targetState) {
            case OFF -> {
                floorMotor.setVoltage(0);
                kickerMotor.setVoltage(0);
            }
            case REVERSE -> {
                floorMotor.setVoltage(-floorMotorShootVolts.get());
                kickerMotor.setVoltage(-kickerMotorShootVolts.get());
            }
            case JIGGLE -> {
                // Sinusoidal oscillation to prevent game piece jams
                double jiggleVolts =
                        Math.sin(2 * Math.PI * jiggleFrequency.get() * (System.currentTimeMillis() / 1000.0))
                                * floorMotorJiggleVolts.get();
                floorMotor.setVoltage(jiggleVolts);
                kickerMotor.setVoltage(0);
            }
            case SHOOT -> {
                floorMotor.setVoltage(floorMotorShootVolts.get());
                kickerMotor.setVoltage(kickerMotorShootVolts.get());
            }
        }

        Logger.recordOutput("IndexerSubsystem/TargetState", this.targetState);
    }
}
