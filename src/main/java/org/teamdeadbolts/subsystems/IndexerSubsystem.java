/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.teamdeadbolts.constants.IndexerConstants;
import org.teamdeadbolts.utils.StatefulSubsystem;
import org.teamdeadbolts.utils.tuning.Refreshable;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class IndexerSubsystem extends StatefulSubsystem<IndexerSubsystem.State> implements Refreshable {
    public enum State {
        OFF,
        JIGGLE,
        SHOOT,
        REVERSE,
    }

    private CANBus canBus = new CANBus("*");
    private TalonFX floorMotor = new TalonFX(IndexerConstants.INDEXER_FLOOR_MOTOR_CAN_ID, canBus);
    private TalonFX kickerMotor = new TalonFX(IndexerConstants.INDEXER_KICKER_MOTOR_CAN_ID, canBus);
    // private DigitalInput ballSensor = new DigitalInput(IndexerConstants.INDEXER_BALL_SENSOR_CHANNEL);

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
            case OFF:
                floorMotor.setVoltage(0);
                kickerMotor.setVoltage(0);
                break;
            case REVERSE:
                floorMotor.setVoltage(-floorMotorShootVolts.get());
                kickerMotor.setVoltage(-kickerMotorShootVolts.get());
                break;
            case JIGGLE:
                double jiggleVolts =
                        Math.sin(2 * Math.PI * jiggleFrequency.get() * (System.currentTimeMillis() / 1000.0))
                                * floorMotorJiggleVolts.get();
                floorMotor.setVoltage(jiggleVolts);
                kickerMotor.setVoltage(0);
                break;
            case SHOOT:
                floorMotor.setVoltage(floorMotorShootVolts.get());
                kickerMotor.setVoltage(kickerMotorShootVolts.get());
                break;
        }
    }
}
