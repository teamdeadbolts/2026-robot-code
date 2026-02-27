/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.teamdeadbolts.commands.DriveCommand;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO;
import org.teamdeadbolts.subsystems.vision.VisionSubsystem;

public class RobotContainer {

    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // private HopperSubsystem hopperSubsystem = new HopperSubsystem();
    private IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    @SuppressWarnings("unused")
    private VisionSubsystem visionSubsystem = new VisionSubsystem(
            swerveSubsystem, new PhotonVisionIO("CenterCam", new Transform3d())
            // new PhotonVisionIO(
            //         "TurretCam", () ->
            // shooterSubsystem.getTurretOffset().plus(VisionConstants.TURRET_CAM_TO_TURRENT)));
            );

    private CommandXboxController primaryController = new CommandXboxController(0);

    private RobotState robotState = RobotState.getInstance();

    // private final AutoFactory autoFactory = new AutoFactory(
    //         robotState.getRobotPose()::toPose2d,
    //         (pose) -> robotState.setEstimatedPose(new Pose3d(pose)),
    //         swerveSubsystem::followTrajectory,
    //         true,
    //         swerveSubsystem);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        robotState.initPoseEstimator(
                new Rotation3d(swerveSubsystem.getGyroRotation()), swerveSubsystem.getModulePositions());

        configureBindings();
    }

    private void configureBindings() {
        // Xbox controllers push "up" = neg valve so invert everything
        swerveSubsystem.setDefaultCommand(new DriveCommand(
                swerveSubsystem,
                primaryController::getLeftY,
                primaryController::getLeftX,
                primaryController::getRightX,
                true));

        shooterSubsystem.setDefaultCommand(
                new RunCommand(() -> shooterSubsystem.setState(ShooterSubsystem.State.OFF), shooterSubsystem));
        // hopperSubsystem.setDefaultCommand(
        //         new RunCommand(() -> hopperSubsystem.setState(HopperSubsystem.State.HOLD), hopperSubsystem));
        intakeSubsystem.setDefaultCommand(
                new RunCommand(() -> intakeSubsystem.setState(IntakeSubsystem.State.STOWED), intakeSubsystem));
        indexerSubsystem.setDefaultCommand(
                new RunCommand(() -> indexerSubsystem.setState(IndexerSubsystem.State.OFF), indexerSubsystem));

        primaryController
                .a()
                .whileTrue(new RunCommand(
                        () -> {
                            // CtreConfigs.init();
                            swerveSubsystem.refreshTuning(false);
                            intakeSubsystem.reconfigure();
                            // shooterSubsystem.reconfigure();
                        },
                        swerveSubsystem));

        primaryController.x().whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));

        primaryController
                .y()
                .whileTrue(new RunCommand(() -> robotState.setEstimatedPose(new Pose3d()), swerveSubsystem));

        // primaryController
        //         .povRight()
        //         .whileTrue(new RunCommand(() -> shooterSubsystem.resetTurrentPosition(), shooterSubsystem));
        primaryController
                .povDown()
                .whileTrue(new RunCommand(
                        () -> {
                            // shooterSubsystem.setState(ShooterSubsystem.State.TEST);
                            intakeSubsystem.setState(IntakeSubsystem.State.SHOOT);
                        },
                        // shooterSubsystem,
                        intakeSubsystem));
        primaryController
                .povUp()
                .whileTrue(new RunCommand(
                        () -> {
                            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
                            shooterSubsystem.setState(ShooterSubsystem.State.TEST);
                        },
                        intakeSubsystem));

        primaryController
                .povLeft()
                .whileTrue(new RunCommand(
                        () -> {
                            intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED);
                        },
                        intakeSubsystem));
        primaryController
                .povRight()
                .whileTrue(new RunCommand(
                        () -> {
                            intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
                        },
                        intakeSubsystem));
        primaryController
                .rightBumper()
                .whileTrue(new RunCommand(
                        () -> {
                            intakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
                        },
                        intakeSubsystem));

        primaryController
                .leftBumper()
                .whileTrue(new RunCommand(
                        () -> {
                            shooterSubsystem.setState(ShooterSubsystem.State.TEST);
                        },
                        shooterSubsystem));

        // primaryController.povLeft().whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
