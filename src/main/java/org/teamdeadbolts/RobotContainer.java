/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.teamdeadbolts.commands.DriveCommand;
// import org.teamdeadbolts.commands.HopperCommand;
// import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.commands.ShootCommand;
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
    // private HopperSubsystem hopperSubsystem = new HopperSubsystem();

    @SuppressWarnings("unused")
    private VisionSubsystem visionSubsystem = new VisionSubsystem(
            swerveSubsystem, new PhotonVisionIO("CenterCam", new Transform3d())
            // new PhotonVisionIO(
            //         "TurretCam", () ->
            // shooterSubsystem.getTurretOffset().plus(VisionConstants.TURRET_CAM_TO_TURRENT)));
            );

    private CommandXboxController primaryController = new CommandXboxController(0);

    private RobotState robotState = RobotState.getInstance();

    private final AutoFactory autoFactory;

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        robotState.initPoseEstimator(
                new Rotation3d(swerveSubsystem.getGyroRotation()), swerveSubsystem.getModulePositions());

        this.autoFactory = new AutoFactory(
                robotState.getRobotPose()::toPose2d,
                (pose) -> robotState.setEstimatedPose(
                        new Pose3d(new Translation3d(pose.getTranslation()), new Rotation3d(pose.getRotation()))),
                swerveSubsystem::followTrajectory,
                true,
                swerveSubsystem);

        configureBindings();
        configureAuto();
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
        intakeSubsystem.setDefaultCommand(
                new RunCommand(() -> intakeSubsystem.setState(IntakeSubsystem.State.OFF), intakeSubsystem));
        indexerSubsystem.setDefaultCommand(
                new RunCommand(() -> indexerSubsystem.setState(IndexerSubsystem.State.OFF), indexerSubsystem));
        //         hopperSubsystem.setDefaultCommand(new HopperCommand(hopperSubsystem, true));

        //        primaryController
        //                .a()
        //                .whileTrue(new RunCommand(
        //                        () -> {
        //                            // CtreConfigs.init();
        //                            // shooterSubsystem.reconfigure();
        //                        },
        //                        swerveSubsystem));

        primaryController.x().whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));

        primaryController
                .y()
                .whileTrue(new RunCommand(() -> robotState.setEstimatedPose(new Pose3d()), swerveSubsystem));

        primaryController.b().whileTrue(getAutonomousCommand());
        //
        //
        primaryController
                .a()
                .whileTrue(new RunCommand(
                        () -> shooterSubsystem.setState(ShooterSubsystem.State.SPINUP), shooterSubsystem));

        primaryController
                .povDown() // Up
                .whileTrue(new ShootCommand(indexerSubsystem, shooterSubsystem, intakeSubsystem));
        primaryController
                .povUp() // Down (retarded)
                .whileTrue(new RunCommand(
                        () -> {
                            shooterSubsystem.setState(ShooterSubsystem.State.ZERO);
                        },
                        shooterSubsystem));

        primaryController
                .povLeft()
                .whileTrue(new RunCommand(
                        () -> {
                            intakeSubsystem.setState(IntakeSubsystem.State.STOWED);
                        },
                        intakeSubsystem));
        primaryController
                .povRight()
                .whileTrue(new RunCommand(
                        () -> {
                            intakeSubsystem.setState(IntakeSubsystem.State.SHOOT);
                        },
                        intakeSubsystem));
        primaryController
                .rightBumper()
                .whileTrue(new RunCommand(
                        () -> {
                            intakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
                        },
                        intakeSubsystem));
        //
        primaryController
                .leftBumper()
                .whileTrue(new RunCommand(
                        () -> {
                            shooterSubsystem.setState(ShooterSubsystem.State.SPINUP);
                            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
                            // intakeSubsystem.setState(IntakeSubsystem.State.SHOOT);
                        },
                        shooterSubsystem,
                        indexerSubsystem
                        // intakeSubsystem
                        ));

        // primaryController.povLeft().whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kReverse));
    }
    //
    private void configureAuto() {
        //        autoFactory.bind(
        //                "Index",
        //                new RunCommand(
        //                        () -> {
        //                            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT);
        //                            System.out.println("Auto Command");
        //                        },
        //                        indexerSubsystem));

        autoChooser.addOption(
                "Test",
                new SequentialCommandGroup(
                        autoFactory.resetOdometry("TestPath"), autoFactory.trajectoryCmd("TestPath")));
    }

    private AutoRoutine testAuto() {
        AutoRoutine routine = autoFactory.newRoutine("testRoutine");
        AutoTrajectory testTraj = routine.trajectory("TestPath");

        routine.active().onTrue(Commands.sequence(testTraj.resetOdometry(), testTraj.cmd()));

        // testTraj.atTime("Index").onTrue(new RunCommand(() ->
        // indexerSubsystem.setState(IndexerSubsystem.State.SHOOT)));
        // testTraj.atTime("StopIndex").onTrue(new RunCommand(() ->
        // indexerSubsystem.setState(IndexerSubsystem.State.OFF)));
        return routine;
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return testAuto().cmd();
    }
}
