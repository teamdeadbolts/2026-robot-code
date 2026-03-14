/* The Deadbolts (C) 2026 */
package org.teamdeadbolts.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamdeadbolts.RobotState;
import org.teamdeadbolts.constants.ZoneConstants;
import org.teamdeadbolts.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final RobotState robotState = RobotState.getInstance();
    private IntakeSubsystem.State prevIntakeState = IntakeSubsystem.State.OFF;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.prevIntakeState = intakeSubsystem.getState();
    }

    @Override
    public void execute() {
        if (isInUpZone(robotState.getRobotPose().toPose2d().getTranslation())
                && intakeSubsystem.getState() != IntakeSubsystem.State.STOWED) {
            intakeSubsystem.setState(IntakeSubsystem.State.HALF_HOLD);
            return;
        }

        if (this.prevIntakeState == IntakeSubsystem.State.OUTTAKE) this.prevIntakeState = IntakeSubsystem.State.OFF;
        this.intakeSubsystem.setState(this.prevIntakeState);
    }

    private boolean isInUpZone(Translation2d robotPose) {
        return ZoneConstants.BLUE_TOP_BUMP_ZONE.contains(robotPose)
                || ZoneConstants.BLUE_BOTTOM_BUMP_ZONE.contains(robotPose)
                || ZoneConstants.RED_TOP_BUMP_ZONE.contains(robotPose)
                || ZoneConstants.RED_BOTTOM_BUMP_ZONE.contains(robotPose);
    }
}
