/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;

public class DefaultShooterCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        if (inSpinUpArea() && indexerSubsystem.hasBall()) {
            shooterSubsystem.setState(ShooterSubsystem.State.SPINUP);
        } else {
            shooterSubsystem.setState(ShooterSubsystem.State.OFF);
        }
    }

    private boolean inSpinUpArea() {
        return false; // TODO: Implement
    }
}
