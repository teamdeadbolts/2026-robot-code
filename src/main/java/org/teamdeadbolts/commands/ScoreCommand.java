/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.SwerveConstants;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.utils.MathUtils;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ScoreCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private HopperSubsystem hopperSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public enum PassingTarget {
        LEFT,
        RIGHT;
    };

    private boolean isPassing;

    private RobotState robotState = RobotState.getInstance();

    private boolean isFinished = false;

    private Timer lastBallTimer = new Timer();

    private Alliance alliance; // Wont change in match

    private final SavedLoggedNetworkNumber maxRPMError = SavedLoggedNetworkNumber.get("Tuning/Scoring/MaxRPMError", 0);
    private final SavedLoggedNetworkNumber earlyShootThreshold =
            SavedLoggedNetworkNumber.get("Tuning/Scoring/EarlyShootThreshold", 2);
    private final SavedLoggedNetworkNumber flushTime = SavedLoggedNetworkNumber.get("Tuning/Scoring/FlushTime", 0.5);

    public ScoreCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            HopperSubsystem hopperSubsystem,
            IntakeSubsystem intakeSubsystem,
            boolean isPassing) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.isPassing = isPassing;

        addRequirements(shooterSubsystem, indexerSubsystem, hopperSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.alliance = DriverStation.getAlliance().get();
        lastBallTimer.reset();
        lastBallTimer.start();
        if (isPassing) {
            shooterSubsystem.setState(ShooterSubsystem.State.PASS);

            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
            intakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
        } else {
            shooterSubsystem.setState(ShooterSubsystem.State.SHOOT);
            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
            hopperSubsystem.setState(HopperSubsystem.State.SLOW_DOWN);
            intakeSubsystem.setState(IntakeSubsystem.State.SHOOT);
        }
    }

    @Override
    public void execute() {
        Translation2d robotTrans = robotState.getRobotPose().toPose2d().getTranslation();
        if (Math.abs(shooterSubsystem.getRPMError()) > maxRPMError.get()
                || (isPassing
                        && MathUtils.inRange(
                                robotTrans.getY(), Units.inchesToMeters(135.594), Units.inchesToMeters(182.594)))) {
            indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        } else {
            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
        }

        if (indexerSubsystem.hasBall()) {
            lastBallTimer.reset();
        }

        // If we are in an alliance shift (not auto or transition), our side is not active, and the shift ends after our
        // early shoot threshold, we are done
        if (!isPassing) {
            if (robotState.getActiveAlliance().isPresent()
                            && DriverStation.getAlliance().get()
                                    != robotState.getActiveAlliance().get()
                            && robotState.getTimeUntilActiveSwitch() > earlyShootThreshold.get()
                    || lastBallTimer.get() > flushTime.get()) {
                isFinished = true;
            }

            if (SwerveConstants.RED_CLOSE_ZONE.contains(robotTrans)
                    || SwerveConstants.BLUE_CLOSE_ZONE.contains(robotTrans)) {
                shooterSubsystem.setState(ShooterSubsystem.State.SPINUP);
            } else {
                shooterSubsystem.setState(ShooterSubsystem.State.SHOOT);
            }

            if ((alliance == Alliance.Blue && !SwerveConstants.BLUE_SCORE_ZONE.contains(robotTrans)
                    || alliance == Alliance.Red && !SwerveConstants.RED_SCORE_ZONE.contains(robotTrans))) {
                isFinished = true;
                CommandScheduler.getInstance()
                        .schedule(new ScoreCommand(
                                shooterSubsystem,
                                indexerSubsystem,
                                hopperSubsystem,
                                intakeSubsystem,
                                true)); // This should work...
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setState(ShooterSubsystem.State.OFF);
        indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        hopperSubsystem.setState(HopperSubsystem.State.HOLD);
        intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
    }

    @Override
    public boolean isFinished() {
        return isFinished; // TODO: Check active hub and if balls
    }
}
