/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

/**
 * Activate the shooter to start shooter, automaticly decides
 * weather to pass or score based off robot location
 */
public class ShootCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;

    private final SavedTunableNumber wheelRpmError = SavedTunableNumber.get("Tuning/ShootCommand/WheelRpmError", 200);

    private final SavedTunableNumber turretPositionError =
            SavedTunableNumber.get("Tuning/ShootCommand/TurretPositionError", 6);

    private boolean fallback;

    public ShootCommand(
            IndexerSubsystem indexerSubsystem,
            ShooterSubsystem shooterSubsystem,
            HopperSubsystem hopperSubsystem,
            boolean fallback) {
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.fallback = fallback;
        addRequirements(indexerSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setTurretFallbackMode(fallback);
    }

    @Override
    public void execute() {
        Translation2d robotLocaion =
                RobotState.getInstance().getRobotPose().toPose2d().getTranslation();
        boolean passing = false;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if ((alliance == Alliance.Blue && !ZoneConstants.BLUE_SCORE_ZONE.contains(robotLocaion))
                || (alliance == Alliance.Red && !ZoneConstants.RED_SCORE_ZONE.contains(robotLocaion))) passing = true;

        shooterSubsystem.setAlternativeHoodMinAngle(hopperSubsystem.getState() == HopperSubsystem.State.UP);

        if (passing) {
            shooterSubsystem.setState(ShooterSubsystem.State.PASS);
        } else {
            shooterSubsystem.setState(ShooterSubsystem.State.SHOOT);
        }
        if (Math.abs(shooterSubsystem.getRPMError()) <= wheelRpmError.get()
                && shooterSubsystem.isPossibleShot()
                && shooterSubsystem.getTurretError() <= Units.degreesToRadians(turretPositionError.get())) {
            feedShooter();
        } else {
            indexerSubsystem.setState(IndexerSubsystem.State.JIGGLE);
        }
        // shooterSubsystem.setState(ShooterSubsystem.State.SPINUP);
    }

    @Override
    public void end(boolean i) {
        indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        shooterSubsystem.setState(ShooterSubsystem.State.APRILTAG_TRACK);
    }

    private void feedShooter() {
        indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
    }
}
