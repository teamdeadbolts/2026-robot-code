/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;

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

    public IntakeCommand(IntakeSubsystem intakeSubsystem, HopperSubsystem hopperSubsystem, Target target) {
        this.intakeSubsystem = intakeSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.target = target;

        addRequirements(intakeSubsystem, hopperSubsystem);
    }

    @Override
    public void initialize() {
        this.prevHopperState = hopperSubsystem.getState();
        if (this.prevHopperState != HopperSubsystem.State.UP && this.target != Target.SHOOT) {
            hopperSubsystem.setState(HopperSubsystem.State.UP);
        }
    }

    @Override
    public void execute() {
        if (hopperSubsystem.lidAtGoal()) {
            switch (target) {
                case INTAKE -> {
                    intakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
                }
                case SHOOT -> {
                    intakeSubsystem.setState(IntakeSubsystem.State.SHOOT);
                }
                case STOW -> {
                    intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
                }
            }
        }

        if (intakeSubsystem.armAtGoal()) {
            this.hopperSubsystem.setState(this.prevHopperState);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED);
        hopperSubsystem.setState(this.prevHopperState);
    }
}
