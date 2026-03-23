/* The Deadbolts (C) 2026 */
package org.teamdeadbolts;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Records the number of calls to a method and sets shouldLog
 * to true after a certain number of calls.
 */
public class PeriodicTasks {
    private static final int LOG_AFTER_N_CALLS = 10;
    private static final int REFRESH_SIGNALS_AFTER_N_CALLS = 10;
    private static final PeriodicTasks INSTANCE = new PeriodicTasks();

    private final AtomicInteger callCount = new AtomicInteger(0);
    private final AtomicBoolean shouldLog = new AtomicBoolean(false);
    private final AtomicBoolean shouldRefreshSignals = new AtomicBoolean(false);

    private PeriodicTasks() {}

    public static PeriodicTasks getInstance() {
        return INSTANCE;
    }

    public void recordCall() {
        callCount.incrementAndGet();
        updateFlag(LOG_AFTER_N_CALLS, shouldLog, false);
        updateFlag(REFRESH_SIGNALS_AFTER_N_CALLS, shouldRefreshSignals, true);
    }

    public boolean shouldLog() {
        return shouldLog.get();
    }

    public boolean shouldRefreshSignals() {
        return shouldRefreshSignals.get();
    }

    private void updateFlag(int callsLimit, AtomicBoolean flag, boolean allowFirstCall) {
        if (allowFirstCall && callCount.get() == 1) {
            flag.set(true);
            return;
        }

        if (callCount.get() % callsLimit == 0) {
            flag.set(true);
            callCount.set(0);
        } else {
            flag.set(false);
        }
    }
}
