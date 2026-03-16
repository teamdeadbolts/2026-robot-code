/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

/**
 * Run a system test routine to test all the mechanisums
 */
public class SystemsTestCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private final Timer stepTimer = new Timer();
    private Step step = Step.SWERVE_FORWARD;

    private boolean isFinished = false;

    // ---- Tunable timings (seconds) ----
    private final SavedLoggedNetworkNumber swerveStepTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/SwerveStepTime", 1.0);
    private final SavedLoggedNetworkNumber swerveSpinTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/SwerveSpinTime", 2.25);
    private final SavedLoggedNetworkNumber swerveAroundTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/SwerveAroundTime", 3.5);

    private final SavedLoggedNetworkNumber hopperMoveTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/HopperMoveTime", 1.0);
    private final SavedLoggedNetworkNumber intakeMoveTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/IntakeMoveTime", 1.0);

    private final SavedLoggedNetworkNumber outtakeTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/OuttakeTime", 3.0);
    private final SavedLoggedNetworkNumber intakeTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/IntakeTime", 3.0);

    private final SavedLoggedNetworkNumber indexerJiggleTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/IndexerJiggleTime", 3.0);
    private final SavedLoggedNetworkNumber indexerForwardTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/IndexerForwardTime", 3.75);

    private final SavedLoggedNetworkNumber turretTrackTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/TurretTrackTime", 6.0);

    private final SavedLoggedNetworkNumber shooterShowoffTime =
            SavedLoggedNetworkNumber.get("Tuning/FullRobotTest/ShooterShowoffTime", 6.0);

    private enum Step {
        // Swerve startup sequence
        SWERVE_FORWARD,
        SWERVE_BACK,
        SWERVE_LEFT,
        SWERVE_RIGHT,
        SWERVE_SPIN,
        SWERVE_AROUND,
        SWERVE_STOP,

        // Hopper + intake
        HOPPER_UP,
        INTAKE_DOWN,
        INTAKE_OUTTAKE,
        INTAKE_INTAKE,
        INTAKE_STOP,

        // Indexer
        INDEXER_JIGGLE,
        INDEXER_FORWARD,
        INDEXER_STOP,

        // Stow
        HOPPER_DOWN,
        INTAKE_IN,

        // Turret april tag tracking
        TURRET_TRACK_TAG,

        // Shooter cool thing
        SHOWOFF_SHOOTER,
        DONE
    }

    public SystemsTestCommand(
            SwerveSubsystem swerveSubsystem,
            HopperSubsystem hopperSubsystem,
            IntakeSubsystem intakeSubsystem,
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem) {

        this.swerveSubsystem = swerveSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(swerveSubsystem, hopperSubsystem, intakeSubsystem, indexerSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;
        step = Step.SWERVE_FORWARD;

        stepTimer.reset();
        stepTimer.start();

        enterStep(step);
    }

    @Override
    public void execute() {

        if (isStepComplete(step)) {
            step = nextStep(step);
            stepTimer.reset();
            stepTimer.start();

            enterStep(step);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Put robot in a neutral state
        try {
            swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), false, false, false);
        } catch (Exception ignored) {
        }

        intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
        indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        shooterSubsystem.setState(ShooterSubsystem.State.OFF);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    // -------------------- Step logic --------------------

    private void enterStep(Step s) {
        switch (s) {
            // ---- SWERVE ----
            case SWERVE_FORWARD:
                swerveSubsystem.drive(new ChassisSpeeds(0, 3, 0), false, false, false);
                break;
            case SWERVE_BACK:
                swerveSubsystem.drive(new ChassisSpeeds(0, -3, 0), false, false, false);
                break;
            case SWERVE_LEFT:
                swerveSubsystem.drive(new ChassisSpeeds(3, 0, 0), false, false, false);
                break;
            case SWERVE_RIGHT:
                swerveSubsystem.drive(new ChassisSpeeds(-3, 0, 0), false, false, false);
                break;
            case SWERVE_SPIN:
                swerveSubsystem.drive(new ChassisSpeeds(0, 0, 3), false, false, false);
                break;
            case SWERVE_AROUND:
                swerveSubsystem.drive(new ChassisSpeeds(3, 3, 3), false, false, false);
                break;
            case SWERVE_STOP:
                swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0), false, false, false);
                break;

            // ---- HOPPER + INTAKE ----
            case HOPPER_UP:
                hopperSubsystem.setState(HopperSubsystem.State.UP);
                break;
            case INTAKE_DOWN:
                intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED);
                break;
            case INTAKE_OUTTAKE:
                intakeSubsystem.setState(IntakeSubsystem.State.OUTTAKE); // your ScoreCommand uses SHOOT as “outtake”
                break;
            case INTAKE_INTAKE:
                intakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
                break;
            case INTAKE_STOP:
                intakeSubsystem.setState(IntakeSubsystem.State.OFF);
                break;

            // ---- INDEXER ----
            case INDEXER_JIGGLE:
                indexerSubsystem.setState(IndexerSubsystem.State.JIGGLE);
                break;
            case INDEXER_FORWARD:
                indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
                break;
            case INDEXER_STOP:
                indexerSubsystem.setState(IndexerSubsystem.State.OFF);
                break;

            // ---- STOW ----
            case HOPPER_DOWN:
                hopperSubsystem.setState(HopperSubsystem.State.DOWN);
                break;
            case INTAKE_IN:
                intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
                break;

            // ---- TURRET TRACK ----
            case TURRET_TRACK_TAG:
                shooterSubsystem.setState(ShooterSubsystem.State.APRILTAG_TRACK);
                break;

            // ---- SHOWOFF ----
            case SHOWOFF_SHOOTER:
                shooterSubsystem.setState(ShooterSubsystem.State.SYSTEMS_TEST);
                break;

            case DONE:
                isFinished = true;
                break;
        }
    }

    private boolean isStepComplete(Step s) {
        double t = stepTimer.get();

        switch (s) {
            // Swerve:
            case SWERVE_FORWARD:
            case SWERVE_BACK:
            case SWERVE_LEFT:
            case SWERVE_RIGHT:
                return t >= swerveStepTime.get();

            case SWERVE_SPIN:
                return t >= swerveSpinTime.get();

            case SWERVE_AROUND:
                return t >= swerveAroundTime.get();

            case SWERVE_STOP:
                return t >= 0.25;

            // Hopper / intake:
            case HOPPER_UP:
                // TODO  hopperSubsystem.isAtTop() use that instead (or with time)
                return t >= hopperMoveTime.get();

            case INTAKE_DOWN:
                // TODO  intakeSubsystem.isDeployed()
                return t >= intakeMoveTime.get();

            case INTAKE_OUTTAKE:
                return t >= outtakeTime.get();

            case INTAKE_INTAKE:
                return t >= intakeTime.get();

            case INTAKE_STOP:
                return t >= 0.15;

            // Indexer
            case INDEXER_JIGGLE:
                return t >= indexerJiggleTime.get();

            case INDEXER_FORWARD:
                return t >= indexerForwardTime.get();

            case INDEXER_STOP:
                return t >= 0.15;

            // Stow
            case HOPPER_DOWN:
                // TODO hopperSubsystem.isAtBottom()
                return t >= hopperMoveTime.get();

            case INTAKE_IN:
                // TODO  intakeSubsystem.isStowed()
                return t >= intakeMoveTime.get();

            // Turret track
            case TURRET_TRACK_TAG:
                return t >= turretTrackTime.get();

            // Showoff:
            case SHOWOFF_SHOOTER:
                return t >= shooterShowoffTime.get();

            case DONE:
                return true;
        }
        return false;
    }

    private Step nextStep(Step s) {
        switch (s) {
            case SWERVE_FORWARD:
                return Step.SWERVE_BACK;
            case SWERVE_BACK:
                return Step.SWERVE_LEFT;
            case SWERVE_LEFT:
                return Step.SWERVE_RIGHT;
            case SWERVE_RIGHT:
                return Step.SWERVE_SPIN;
            case SWERVE_SPIN:
                return Step.SWERVE_AROUND;
            case SWERVE_AROUND:
                return Step.SWERVE_STOP;

            case SWERVE_STOP:
                return Step.HOPPER_UP;
            case HOPPER_UP:
                return Step.INTAKE_DOWN;
            case INTAKE_DOWN:
                return Step.INTAKE_OUTTAKE;
            case INTAKE_OUTTAKE:
                return Step.INTAKE_INTAKE;
            case INTAKE_INTAKE:
                return Step.INTAKE_STOP;

            case INTAKE_STOP:
                return Step.INDEXER_JIGGLE;
            case INDEXER_JIGGLE:
                return Step.INDEXER_FORWARD;
            case INDEXER_FORWARD:
                return Step.INDEXER_STOP;

            case INDEXER_STOP:
                return Step.HOPPER_DOWN;
            case HOPPER_DOWN:
                return Step.INTAKE_IN;

            case INTAKE_IN:
                return Step.TURRET_TRACK_TAG;

            case TURRET_TRACK_TAG:
                return Step.SHOWOFF_SHOOTER;
            case SHOWOFF_SHOOTER:
                return Step.DONE;

            case DONE:
                return Step.DONE;
        }
        return Step.DONE;
    }
}
