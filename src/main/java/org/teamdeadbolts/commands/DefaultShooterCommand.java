/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;

public class DefaultShooterCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private RobotState robotState = RobotState.getInstance();

    public DefaultShooterCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotState.getRobotPose().toPose2d();
        // if (SwerveConstants)

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
