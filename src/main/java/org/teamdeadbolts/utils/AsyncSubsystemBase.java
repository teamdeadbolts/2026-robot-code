/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.teamdeadbolts.RobotState;

/**
 * Runs periodicAsync() in a separate thread.
 */
public abstract class AsyncSubsystemBase extends SubsystemBase {

    public void periodic() {
        RobotState.getInstance().execAsync(this::periodicAsync);
    }

    public void periodicAsync() {}
}
