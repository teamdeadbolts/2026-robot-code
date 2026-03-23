/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.teamdeadbolts.constants.IndexerConstants;
import org.teamdeadbolts.utils.PeriodicTasks;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

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

    private final StatusSignal<Current> floorMotorCurrentSignal = floorMotor.getSupplyCurrent();
    private final StatusSignal<Current> kickerMotorCurrentSignal = kickerMotor.getSupplyCurrent();
    private final StatusSignal<Voltage> floorMotorVoltageSignal = floorMotor.getSupplyVoltage();

    private final List<BaseStatusSignal> signals =
            List.of(floorMotorCurrentSignal, kickerMotorCurrentSignal, floorMotorVoltageSignal, floorMotorVoltageSignal);

    /* --- Tuning Parameters --- */
    private final SavedTunableNumber floorMotorIntakeVolts =
            SavedTunableNumber.get("Tuning/Indexer/IndexerFloorMotorIntakeVolts", 6.0);
    private final SavedTunableNumber floorMotorShootVolts =
            SavedTunableNumber.get("Tuning/Indexer/IndexerFloorMotorShootVolts", 6.0);
    private final SavedTunableNumber floorMotorJiggleVolts =
            SavedTunableNumber.get("Tuning/Indexer/IndexerFloorMotorJiggleVolts", 3.0);
    private final SavedTunableNumber kickerMotorShootVolts =
            SavedTunableNumber.get("Tuning/Indexer/IndexerKickerMotorShootVolts", 6.0);
    private final SavedTunableNumber jiggleFrequency =
            SavedTunableNumber.get("Tuning/Indexer/IndexerJiggleFrequency", 1.0);

    public IndexerSubsystem() {
        super(State.OFF);
        refresh();
    }

    @Override
    public void refresh() {
        IndexerConstants.init();
        floorMotor.getConfigurator().apply(IndexerConstants.INDEXER_FLOOR_MOTOR_CONFIG);
        kickerMotor.getConfigurator().apply(IndexerConstants.INDEXER_KICKER_MOTOR_CONFIG);
    }

    @Override
    protected void onStateChange(final State to, final State from) {}

    public List<BaseStatusSignal> getSignals() {
        return signals;
    }

    @Override
    public void subsystemPeriodic() {
        if (PeriodicTasks.getInstance().shouldRefreshSignals()) {
            BaseStatusSignal.refreshAll(signals);
        }
        switch (targetState) {
            case OFF -> {
                floorMotor.setVoltage(0);
                kickerMotor.setVoltage(0);
            }
            case REVERSE, JIGGLE -> {
                // Sinusoidal oscillation to prevent game piece jams
                final double jiggleVolts =
                        Math.sin(2 * Math.PI * jiggleFrequency.get() * (System.currentTimeMillis() / 1000.0))
                                * floorMotorJiggleVolts.get();
                floorMotor.setVoltage(jiggleVolts);
                // kickerMotor.setVoltage(0);
                if (targetState == State.REVERSE) kickerMotor.setVoltage(-floorMotorIntakeVolts.get());
                else kickerMotor.setVoltage(0);
            }
            case SHOOT -> {
                floorMotor.setVoltage(floorMotorShootVolts.get());
                kickerMotor.setVoltage(kickerMotorShootVolts.get());
            }
        }

        if (PeriodicTasks.getInstance().shouldLog()) {
            Logger.recordOutput("IndxerSubsystem/Floor/OutputVolts", floorMotorVoltageSignal.getValueAsDouble());
            // Current
            Logger.recordOutput("Debug/Current/Indexer/Floor", floorMotorCurrentSignal.getValueAsDouble());
            Logger.recordOutput("Debug/Current/Indexer/Kicker", kickerMotorCurrentSignal.getValueAsDouble());
        }
    }
}
