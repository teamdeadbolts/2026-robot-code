/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import org.teamdeadbolts.commands.DefaultIntakeCommand;
import org.teamdeadbolts.commands.DefaultShooterCommand;
import org.teamdeadbolts.commands.DriveCommand;
import org.teamdeadbolts.commands.IntakeCommand;
import org.teamdeadbolts.commands.ShootCommand;
import org.teamdeadbolts.constants.VisionConstants;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO;
import org.teamdeadbolts.subsystems.vision.VisionSubsystem;
import org.teamdeadbolts.utils.StatefulSubsystem.Priority;
import org.teamdeadbolts.utils.tuning.SavedTunableNumber;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final HopperSubsystem hopperSubsystem = new HopperSubsystem();

    private VisionSubsystem visionSubsystem = new VisionSubsystem(
            swerveSubsystem,
            new PhotonVisionIO("Left Cam", VisionConstants.LEFT_CAM_TRANSFORM),
            new PhotonVisionIO("Right Cam", VisionConstants.RIGHT_CAM_TRANSFORM),
            new PhotonVisionIO("Back Cam", VisionConstants.BACK_CAM_TRANSFORM),
            new PhotonVisionIO("Turret Cam", () -> shooterSubsystem
                    .getRobotRelativeTurretTransform()
                    .plus(VisionConstants.TURRET_CAM_TO_TURRET)));

    private CommandXboxController primaryController = new CommandXboxController(0);
    private CommandXboxController secondaryController = new CommandXboxController(1);

    private RobotState robotState = RobotState.getInstance();

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final SavedTunableNumber pathplannerTransKp = SavedTunableNumber.get("Tuning/Pathplanner/Trans/kP", 0.0);
    private final SavedTunableNumber pathplannerTransKi = SavedTunableNumber.get("Tuning/Pathplanner/Trans/kI", 0.0);
    private final SavedTunableNumber pathplannerTransKd = SavedTunableNumber.get("Tuning/Pathplanner/Trans/kD", 0.0);

    private final SavedTunableNumber pathplannerRotKp = SavedTunableNumber.get("Tuning/Pathplanner/Rot/kP", 0.0);
    private final SavedTunableNumber pathplannerRotKi = SavedTunableNumber.get("Tuning/Pathplanner/Rot/kI", 0.0);
    private final SavedTunableNumber pathplannerRotKd = SavedTunableNumber.get("Tuning/Pathplanner/Rot/kD", 0.0);

    private final SavedTunableNumber fastDriveScaler = SavedTunableNumber.get("Tuning/Drive/FastDriveScaler", 1.5);
    private final SavedTunableNumber slowDriveScaler = SavedTunableNumber.get("Tuning/Drive/SlowDriveScaler", 0.28);
    private final SavedTunableNumber shootDriveScaler = SavedTunableNumber.get("Tuning/Drive/ShootDriveScaler", 0.1);
    private final SavedTunableNumber passDriveScaler = SavedTunableNumber.get("Tuning/Drive/PassDriveScaler", 0.28);

    public RobotContainer() {
        robotState.initPoseEstimator(
                new Rotation3d(swerveSubsystem.getGyroRotation()), swerveSubsystem.getModulePositions());

        configureAuto();
        configureBindings();
    }

    private void configureBindings() {
        // Xbox controllers push "up" = neg valve so invert everything
        swerveSubsystem.setDefaultCommand(new DriveCommand(
                swerveSubsystem,
                shooterSubsystem,
                primaryController::getLeftY,
                primaryController::getLeftX,
                primaryController::getRightX,
                true,
                () -> 1));

        shooterSubsystem.setDefaultCommand(new DefaultShooterCommand(shooterSubsystem));
        intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(intakeSubsystem));

        indexerSubsystem.setDefaultCommand(new RunCommand(
                () -> indexerSubsystem.setState(IndexerSubsystem.State.OFF, Priority.LOW), indexerSubsystem));

        // Primary controller
        primaryController
                .leftTrigger(0.4)
                .whileTrue(new DriveCommand(
                        swerveSubsystem,
                        shooterSubsystem,
                        primaryController::getLeftY,
                        primaryController::getLeftX,
                        primaryController::getRightX,
                        true,
                        fastDriveScaler::get));
        primaryController
                .rightTrigger(0.4)
                .whileTrue(new ParallelCommandGroup(new DriveCommand(
                        swerveSubsystem,
                        shooterSubsystem,
                        primaryController::getLeftY,
                        primaryController::getLeftX,
                        primaryController::getRightX,
                        true,
                        slowDriveScaler::get)));

        primaryController.rightBumper().whileTrue(new IntakeCommand(intakeSubsystem, IntakeCommand.Target.INTAKE));
        primaryController.leftBumper().whileTrue(new IntakeCommand(intakeSubsystem, IntakeCommand.Target.STOW));

        primaryController.b().whileTrue(new RunCommand((() -> {
            for (PhotonVisionIO camera : visionSubsystem.getCameras()) {
                camera.findTransform();
            }
        })));

        primaryController
                .b()
                .whileTrue(new RunCommand(
                        () -> shooterSubsystem.setState(ShooterSubsystem.State.TEST, Priority.NORMAL),
                        shooterSubsystem));

        primaryController.x().whileTrue(new RunCommand(() -> swerveSubsystem.resetGyro(), swerveSubsystem));

        primaryController
                .a()
                .whileTrue(
                        new RunCommand(() -> intakeSubsystem.setState(IntakeSubsystem.State.OUTTAKE, Priority.NORMAL)));
        primaryController
                .y()
                .whileTrue(new RunCommand(
                        () -> {
                            shooterSubsystem.setState(ShooterSubsystem.State.TEST, Priority.NORMAL);
                            indexerSubsystem.setState(IndexerSubsystem.State.SHOOT, Priority.NORMAL);
                        },
                        shooterSubsystem));

        primaryController.povDown().whileTrue(new RunCommand(() -> {
            shooterSubsystem.setState(ShooterSubsystem.State.ZERO, Priority.HIGH);
        }));

        // Secondary Controller
        secondaryController
                .povUp()
                .whileTrue(new RunCommand(
                        () -> hopperSubsystem.setState(HopperSubsystem.State.UP, Priority.NORMAL), hopperSubsystem));
        secondaryController
                .povDown()
                .whileTrue(new RunCommand(
                        () -> hopperSubsystem.setState(HopperSubsystem.State.DOWN, Priority.NORMAL), hopperSubsystem));
        secondaryController
                .rightBumper()
                .whileTrue(new RunCommand(
                        () -> indexerSubsystem.setState(IndexerSubsystem.State.JIGGLE, Priority.NORMAL),
                        indexerSubsystem));
        secondaryController.leftTrigger(0.4).whileTrue(new IntakeCommand(intakeSubsystem, IntakeCommand.Target.SHOOT));
        secondaryController
                .leftBumper()
                .whileTrue(new RunCommand(
                        () -> intakeSubsystem.setState(IntakeSubsystem.State.INTAKE, Priority.NORMAL),
                        intakeSubsystem));
        secondaryController
                .a()
                .whileTrue(new ParallelCommandGroup(
                        new DriveCommand(
                                swerveSubsystem,
                                shooterSubsystem,
                                primaryController::getLeftY,
                                primaryController::getLeftX,
                                primaryController::getRightX,
                                true,
                                () -> shooterSubsystem.getState() == ShooterSubsystem.State.PASS
                                        ? passDriveScaler.get()
                                        : shootDriveScaler.get()),
                        new ShootCommand(indexerSubsystem, shooterSubsystem, hopperSubsystem, false)));
        secondaryController
                .povLeft()
                .whileTrue(new RunCommand(
                        () -> {
                            indexerSubsystem.setState(IndexerSubsystem.State.REVERSE, Priority.NORMAL);
                            intakeSubsystem.setState(IntakeSubsystem.State.OUTTAKE, Priority.NORMAL);
                        },
                        intakeSubsystem,
                        indexerSubsystem));
    }

    private void configureAuto() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        RobotState state = RobotState.getInstance();

        new EventTrigger("Index")
                .onTrue(new RunCommand(
                        () -> indexerSubsystem.setState(IndexerSubsystem.State.JIGGLE, Priority.NORMAL),
                        indexerSubsystem));
        new EventTrigger("StopIndex")
                .onTrue(new RunCommand(
                        () -> indexerSubsystem.setState(IndexerSubsystem.State.OFF, Priority.NORMAL),
                        indexerSubsystem));
        new EventTrigger("Intake")
                .onTrue(new RunCommand(
                        () -> intakeSubsystem.setState(IntakeSubsystem.State.INTAKE, Priority.NORMAL),
                        intakeSubsystem));
        new EventTrigger("StopIntake")
                .onTrue(new RunCommand(
                        () -> intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED, Priority.NORMAL),
                        intakeSubsystem));
        new EventTrigger("Shoot").onTrue(new ShootCommand(indexerSubsystem, shooterSubsystem, hopperSubsystem, false));
        new EventTrigger("IntakeShoot")
                .onTrue(new RunCommand(() -> intakeSubsystem.setState(IntakeSubsystem.State.SHOOT, Priority.NORMAL)));

        new EventTrigger("SpinUp")
                .onTrue(new RunCommand(
                        () -> shooterSubsystem.setState(ShooterSubsystem.State.SPINUP, Priority.NORMAL),
                        shooterSubsystem));
        new EventTrigger("StopShoot")
                .onTrue(new RunCommand(
                        () -> shooterSubsystem.setState(ShooterSubsystem.State.OFF, Priority.NORMAL),
                        shooterSubsystem));
        new EventTrigger("HopperUp")
                .onTrue(new RunCommand(
                        () -> hopperSubsystem.setState(HopperSubsystem.State.UP, Priority.NORMAL), hopperSubsystem));
        new EventTrigger("HopperDown")
                .onTrue(new RunCommand(
                        () -> hopperSubsystem.setState(HopperSubsystem.State.DOWN, Priority.NORMAL), hopperSubsystem));
        new EventTrigger("Outtake")
                .onTrue(new RunCommand(() -> intakeSubsystem.setState(IntakeSubsystem.State.OUTTAKE, Priority.NORMAL)));

        AutoBuilder.configure(
                () -> state.getRobotPose().toPose2d(),
                (pose) -> state.setEstimatedPose(new Pose3d(pose)),
                () -> state.getRobotRelativeRobotVelocities(),
                (speeds, _feedforwards) -> {
                    swerveSubsystem.drive(speeds, false, false, false);
                },
                new PPHolonomicDriveController(
                        new PIDConstants(pathplannerTransKp.get(), pathplannerTransKi.get(), pathplannerTransKd.get()),
                        new PIDConstants(pathplannerRotKp.get(), pathplannerRotKi.get(), pathplannerRotKd.get())),
                config,
                () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) return alliance.get() == DriverStation.Alliance.Red;
                    return false;
                },
                swerveSubsystem);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
