/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.utils.StatefulSubsystem.Priority;

/**
 * Intake balls
 */
public class IntakeCommand extends Command {
    public enum Target {
        STOW,
        INTAKE,
        SHOOT;
    }

    private IntakeSubsystem intakeSubsystem;
    private HopperSubsystem hopperSubsystem;
    private Target target;
    private HopperSubsystem.State prevHopperState;
    private boolean hopperUp = false;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem, Target target) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.target = target;
    }

    @Override
    public void initialize() {
        this.prevHopperState = hopperSubsystem.getState();
        // boolean needHopper = this.prevHopperState != HopperSubsystem.State.UP && this.target != Target.SHOOT &&
        // ((this.target == Target.S) || ());
        hopperUp = true;

        if (this.prevHopperState != HopperSubsystem.State.UP
                && this.target != Target.SHOOT
                && ((target == Target.STOW && intakeSubsystem.getState() != IntakeSubsystem.State.STOWED)
                        || (target == Target.INTAKE
                                && intakeSubsystem.getState() != IntakeSubsystem.State.INTAKE
                                && intakeSubsystem.getState() != IntakeSubsystem.State.DEPLOYED))) {
            hopperSubsystem.setState(HopperSubsystem.State.UP, Priority.NORMAL);
            hopperUp = false;
        }
    }

    @Override
    public void execute() {
        if (hopperSubsystem.lidAtGoal()) {
            hopperUp = true;
            switch (target) {
                case INTAKE -> {
                    intakeSubsystem.setState(IntakeSubsystem.State.INTAKE, Priority.NORMAL);
                }
                case SHOOT -> {
                    intakeSubsystem.setState(IntakeSubsystem.State.SHOOT, Priority.NORMAL);
                }
                case STOW -> {
                    intakeSubsystem.setState(IntakeSubsystem.State.STOWED, Priority.NORMAL);
                }
            }
        }

        if (intakeSubsystem.armAtGoal() && hopperUp) {
            this.hopperSubsystem.setState(this.prevHopperState, Priority.NORMAL);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (this.target == Target.INTAKE || this.target == Target.SHOOT)
            intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED, Priority.NORMAL);
        hopperSubsystem.setState(this.prevHopperState, Priority.NORMAL);
    }
}
