/* The Deadbolts (C) 2026 */
package org.teamdeadbolts;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Records the number of calls to a method and sets shouldLog
 * to true after a certain number of calls.
 */
public class PeriodicLogger {
    private static final int LOG_AFTER_N_CALLS = 10;
    private static final PeriodicLogger INSTANCE = new PeriodicLogger();

    private final AtomicInteger callCount = new AtomicInteger(0);
    private final AtomicBoolean shouldLog = new AtomicBoolean(false);

    private PeriodicLogger() {}

    public static PeriodicLogger getInstance() {
        return INSTANCE;
    }

    public void recordCall() {
        callCount.incrementAndGet();
        if (callCount.get() % LOG_AFTER_N_CALLS == 0) {
            shouldLog.set(true);
            callCount.set(0);
        } else {
            shouldLog.set(false);
        }
    }

    public boolean shouldLog() {
        return shouldLog.get();
    }
}
