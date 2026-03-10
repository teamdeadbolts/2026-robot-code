/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.teamdeadbolts.utils.tuning.ConfigManager;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotState robotState = RobotState.getInstance();

    private final RobotContainer robotContainer;

    public Robot() {
        // Setup advantage kit
        Logger.recordMetadata("ProjectName", "2026 Robot");
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            setUseTiming(false);
            Logger.addDataReceiver(new NT4Publisher());

        //     String logPath = LogFileUtil.findReplayLog();
        //     Logger.setReplaySource(new WPILOGReader(logPath));
        //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
        LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
        Logger.start();

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void robotInit() {
        ConfigManager.getInstance().init();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        robotState.resetTimer();
    }

    @Override
    public void teleopPeriodic() {
        robotState.updateActiveAlliance();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}

