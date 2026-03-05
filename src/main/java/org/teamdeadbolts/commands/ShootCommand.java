/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class ShootCommand extends Command {
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    private final SavedLoggedNetworkNumber rpmErrorTol =
            SavedLoggedNetworkNumber.get("Tuning/ShootCommand/rpmErrorTol", 200);

    public ShootCommand(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        //        if (shooterSubsystem.getRPMError() <= rpmErrorTol.get()) {
        //            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
        //        } else {
        //            indexerSubsystem.setState(IndexerSubsystem.State.OFF);
        //        }
        indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);

        shooterSubsystem.setState(ShooterSubsystem.State.SPINUP);
    }
}
