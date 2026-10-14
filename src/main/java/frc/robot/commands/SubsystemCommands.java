package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class SubsystemCommands extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final DoubleSupplier fordwardInput;
  private final DoubleSupplier leftInput;

  public SubsystemCommands(
      Drive drive,
      Shooter shooter,
      Indexer indexer,
      DoubleSupplier fordwardInput,
      DoubleSupplier leftInput) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.fordwardInput = fordwardInput;
    this.leftInput = leftInput;
  }

  // Comando de autoapuntado y disparo
  public Command aimAndShoot(Translation2d landMark) {
    final AimAndDriveCommand aimAndDriveCommand =
        new AimAndDriveCommand(drive, fordwardInput, leftInput, landMark);
    final PrepareShotCommand prepareShotCommand =
        new PrepareShotCommand(shooter, () -> drive.getPose(), landMark);
    return Commands.parallel(
        aimAndDriveCommand,
        prepareShotCommand,
        Commands.waitSeconds(1).andThen(Commands.runOnce(() -> indexer.set(-1), indexer)));
  } // tirale bien chido
  // fermayen?sarabia?fundita
  // fermayenrunningbackmexicanodeborregosmonterrey

  // Comando para recoger combustible
  public Command intakeing() {
    return Commands.parallel(
        Commands.runOnce(() -> shooter.setRPM(5000), shooter),
        Commands.runOnce(() -> indexer.set(1), indexer));
  }

  // Comando de disparo en autonomo sin auto apuntado
  public Command freeFire(Translation2d landMark) {
    final PrepareShotCommand prepareShotCommand =
        new PrepareShotCommand(shooter, () -> drive.getPose(), landMark);
    return Commands.parallel(
        prepareShotCommand,
        Commands.waitSeconds(1).andThen(Commands.runOnce(() -> indexer.feed(), indexer)));
  }

  public Command stop() {
    return Commands.parallel(
        Commands.runOnce(() -> shooter.setRPM(0), shooter),
        Commands.runOnce(() -> indexer.set(0), indexer));
  }
}
