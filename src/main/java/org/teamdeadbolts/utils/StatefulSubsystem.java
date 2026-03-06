/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public abstract class StatefulSubsystem<S extends Enum<S>> extends SubsystemBase {
    @AutoLogOutput
    protected S targetState;

    public final void setState(S newState) {
        if (this.targetState != newState) {
            onStateChange(newState, targetState);
            targetState = newState;
        }
    }

    public S getState() {
        return targetState;
    }

    protected abstract void onStateChange(S from, S to);
}
