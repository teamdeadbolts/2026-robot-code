/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    private Target target;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, Target target) {
        this.intakeSubsystem = intakeSubsystem;
        this.target = target;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
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

    @Override
    public void end(boolean interrupted) {
        if (this.target == Target.INTAKE || this.target == Target.SHOOT) {
            System.out.println("Hello?");
            intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED, Priority.HIGH);
        }
    }
}
