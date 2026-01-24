/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.ShooterSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;
import org.teamdeadbolts.constants.SwerveConstants;

public class ScoreCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private HopperSubsystem hopperSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public enum PassingTarget {
        LEFT,
        RIGHT;
    };

    private PassingTarget passingTarget;
    private boolean isPassing;

    private RobotState robotState = RobotState.getInstance();

    private boolean isFinished = false;

    private Timer lastBallTimer = new Timer();

    private Alliance alliance; // Wont change in match


    private SavedLoggedNetworkNumber maxRPMError = SavedLoggedNetworkNumber.get("Tuning/Scoring/MaxRPMError", 0);
    private SavedLoggedNetworkNumber earlyShootThreshold =
            SavedLoggedNetworkNumber.get("Tuning/Scoring/EarlyShootThreshold", 2);
    private SavedLoggedNetworkNumber flushTime = SavedLoggedNetworkNumber.get("Tuning/Scoring/FlushTime", 0.5);

    public ScoreCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            HopperSubsystem hopperSubsystem,
            IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.isPassing = false;

        addRequirements(shooterSubsystem, indexerSubsystem, hopperSubsystem, intakeSubsystem);
    }

    public ScoreCommand(
            ShooterSubsystem shooterSubsystem,
            IndexerSubsystem indexerSubsystem,
            HopperSubsystem hopperSubsystem,
            IntakeSubsystem intakeSubsystem,
            PassingTarget passingTarget) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.passingTarget = passingTarget;
        this.isPassing = true;

        addRequirements(shooterSubsystem, indexerSubsystem, hopperSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.alliance = DriverStation.getAlliance().get();
        lastBallTimer.reset();
        lastBallTimer.start();
        if (isPassing) {
            if (passingTarget == PassingTarget.LEFT) {
                shooterSubsystem.setState(ShooterSubsystem.State.PASS_LEFT);
            } else {
                shooterSubsystem.setState(ShooterSubsystem.State.PASS_RIGHT);
            }

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
        if (Math.abs(shooterSubsystem.getRPMError()) > maxRPMError.get()) {
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

            if ((alliance == Alliance.Blue && !SwerveConstants.BLUE_SCORE_ZONE.contains(robotTrans) || alliance == Alliance.Red && !SwerveConstants.RED_SCORE_ZONE.contains(robotTrans))) {
                isFinished = true;
                CommandScheduler.getInstance().schedule(new ScoreCommand(shooterSubsystem, indexerSubsystem, hopperSubsystem, intakeSubsystem, PassingTarget.LEFT)); // This should work...
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
