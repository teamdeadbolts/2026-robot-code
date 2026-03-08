/* The Deadbolts (C) 2025 */
package org.teamdeadbolts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import org.teamdeadbolts.commands.DriveCommand;
import org.teamdeadbolts.commands.ShootCommand;
import org.teamdeadbolts.subsystems.HopperSubsystem;
import org.teamdeadbolts.subsystems.IndexerSubsystem;
import org.teamdeadbolts.subsystems.IntakeSubsystem;
import org.teamdeadbolts.subsystems.drive.SwerveSubsystem;
import org.teamdeadbolts.subsystems.shooter.ShooterSubsystem;
import org.teamdeadbolts.subsystems.vision.PhotonVisionIO;
import org.teamdeadbolts.subsystems.vision.VisionSubsystem;
import org.teamdeadbolts.utils.tuning.SavedLoggedNetworkNumber;

public class RobotContainer {

    private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // private HopperSubsystem hopperSubsystem = new HopperSubsystem();
    private IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private HopperSubsystem hopperSubsystem = new HopperSubsystem();

    @SuppressWarnings("unused")
    private VisionSubsystem visionSubsystem = new VisionSubsystem(
            swerveSubsystem, new PhotonVisionIO("CenterCam", new Transform3d())
            // new PhotonVisionIO(
            //         "TurretCam", () ->
            // shooterSubsystem.getTurretOffset().plus(VisionConstants.TURRET_CAM_TO_TURRET)));
            );

    private CommandXboxController primaryController = new CommandXboxController(0);

    private RobotState robotState = RobotState.getInstance();

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final SavedLoggedNetworkNumber pathplannerTransKp =
            SavedLoggedNetworkNumber.get("Tuning/Pathplanner/Trans/kP", 0.0);
    private final SavedLoggedNetworkNumber pathplannerTransKi =
            SavedLoggedNetworkNumber.get("Tuning/Pathplanner/Trans/kI", 0.0);
    private final SavedLoggedNetworkNumber pathplannerTransKd =
            SavedLoggedNetworkNumber.get("Tuning/Pathplanner/Trans/kD", 0.0);

    private final SavedLoggedNetworkNumber pathplannerRotKp =
            SavedLoggedNetworkNumber.get("Tuning/Pathplanner/Rot/kP", 0.0);
    private final SavedLoggedNetworkNumber pathplannerRotKi =
            SavedLoggedNetworkNumber.get("Tuning/Pathplanner/Rot/kI", 0.0);
    private final SavedLoggedNetworkNumber pathplannerRotKd =
            SavedLoggedNetworkNumber.get("Tuning/Pathplanner/Rot/kD", 0.0);

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
                primaryController::getLeftY,
                primaryController::getLeftX,
                primaryController::getRightX,
                true,
                false,
                false));

        shooterSubsystem.setDefaultCommand(
                new RunCommand(() -> shooterSubsystem.setState(ShooterSubsystem.State.OFF), shooterSubsystem));
        intakeSubsystem.setDefaultCommand(
                new RunCommand(() -> intakeSubsystem.setState(IntakeSubsystem.State.OFF), intakeSubsystem));
        indexerSubsystem.setDefaultCommand(
                new RunCommand(() -> indexerSubsystem.setState(IndexerSubsystem.State.OFF), indexerSubsystem));
        hopperSubsystem.setDefaultCommand(
                new RunCommand(() -> hopperSubsystem.setState(HopperSubsystem.State.HOLD), hopperSubsystem));
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

        primaryController
                .rightTrigger(0.4)
                .whileTrue(new DriveCommand(
                        swerveSubsystem,
                        primaryController::getLeftY,
                        primaryController::getLeftX,
                        primaryController::getRightX,
                        true,
                        true,
                        false));
        // Fast button

        primaryController
                .a()
                .whileTrue(new RunCommand(
                        () -> shooterSubsystem.setState(ShooterSubsystem.State.SPINUP), shooterSubsystem));

        primaryController
                .povUp() // Down
                .whileTrue(new RunCommand(
                        () -> {
                            hopperSubsystem.setState(HopperSubsystem.State.DOWN);
                        },
                        hopperSubsystem));

        primaryController
                .povDown() // up
                .whileTrue(new RunCommand(
                        () -> {
                            intakeSubsystem.setState(IntakeSubsystem.State.DEPLOYED);
                        },
                        intakeSubsystem));

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
                            hopperSubsystem.setState(HopperSubsystem.State.UP);
                        },
                        hopperSubsystem));
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
                .whileTrue(new ParallelCommandGroup(
                        new ShootCommand(indexerSubsystem, shooterSubsystem, intakeSubsystem, hopperSubsystem),
                        new DriveCommand(
                                swerveSubsystem,
                                primaryController::getLeftY,
                                primaryController::getLeftX,
                                primaryController::getRightX,
                                true,
                                false,
                                true)));

        // primaryController.povLeft().whileTrue(swerveSubsystem.runDriveQuasiTest(Direction.kReverse));
    }
    //
    private void configureAuto() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        RobotState state = RobotState.getInstance();

        AutoBuilder.configure(
                state.getRobotPose()::toPose2d,
                (pose) -> state.setEstimatedPose(new Pose3d(pose)),
                state::getRobotRelativeRobotVelocities,
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
    }

    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        // return testAuto().cmd();
        return new RunCommand(() -> {});
    }
}
