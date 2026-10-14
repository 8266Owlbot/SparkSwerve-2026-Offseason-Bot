// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Indexer indexer;

  private final SubsystemCommands subsystemCommands;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));

        shooter = new Shooter();
        indexer = new Indexer();

        subsystemCommands =
            new SubsystemCommands(
                drive,
                shooter,
                indexer,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));

        shooter = new Shooter();
        indexer = new Indexer();

        subsystemCommands =
            new SubsystemCommands(
                drive,
                shooter,
                indexer,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});

        shooter = new Shooter();
        indexer = new Indexer();

        subsystemCommands =
            new SubsystemCommands(
                drive,
                shooter,
                indexer,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX());
        break;
    }

    NamedCommands.registerCommand(
        "disparar al hub 1", Commands.runOnce(() -> shooter.setRPM(4500), shooter).withTimeout(5));
    NamedCommands.registerCommand(
        "disparar al hub 2", Commands.runOnce(() -> shooter.setRPM(4000), shooter).withTimeout(5));
    NamedCommands.registerCommand(
        "disparar al hub con auto apuntado",
        subsystemCommands.aimAndShoot(Landmarks.hubPosition()).withTimeout(4));
    NamedCommands.registerCommand("detener", subsystemCommands.stop());
    NamedCommands.registerCommand(
        "recoger",
        subsystemCommands.intakeing().withTimeout(7).finallyDo(() -> subsystemCommands.stop()));
    NamedCommands.registerCommand("llenar", Commands.run(() -> indexer.set(-1)).withTimeout(4));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Auto aim hub command
    controller
        .rightTrigger()
        .onTrue((subsystemCommands.aimAndShoot(Landmarks.hubPosition())))
        .onFalse(subsystemCommands.stop());

    controller
        .leftTrigger()
        .onTrue(
            Commands.parallel(
                Commands.run(() -> shooter.setRPM(4100), shooter),
                Commands.waitSeconds(1).andThen(Commands.run(() -> indexer.set(-1), indexer))))
        .onFalse(
            Commands.parallel(
                Commands.runOnce(() -> shooter.setRPM(0), shooter),
                Commands.runOnce(() -> indexer.set(0), indexer)));

    controller.leftBumper().whileTrue(subsystemCommands.intakeing());

    // Auto aim right trench command
    controller.leftBumper().whileTrue(subsystemCommands.aimAndShoot(Landmarks.allianceLeftZone()));

    // Auto aim left trench command
    controller
        .rightBumper()
        .whileTrue(subsystemCommands.aimAndShoot(Landmarks.allianceRightZone()))
        .whileTrue(subsystemCommands.stop());

    controller
        .a()
        .whileTrue(Commands.run(() -> shooter.setRPM(5000), shooter))
        .whileFalse(Commands.run(() -> shooter.setRPM(0), shooter));

    controller
        .a()
        .whileTrue(Commands.run(() -> indexer.set(1), indexer))
        .whileFalse(Commands.run(() -> indexer.set(0), indexer));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.povUp().whileTrue(shooter.sysIdQuasystic(SysIdRoutine.Direction.kForward));
    controller.povDown().whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
