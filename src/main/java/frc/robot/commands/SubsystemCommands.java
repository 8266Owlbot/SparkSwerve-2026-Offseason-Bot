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

  public Command aimAndShoot(Translation2d landMark) {
    final AimAndDriveCommand aimAndDriveCommand =
        new AimAndDriveCommand(drive, fordwardInput, leftInput, landMark);
    final PrepareShotCommand prepareShotCommand =
        new PrepareShotCommand(shooter, () -> drive.getPose(), landMark);
    return Commands.parallel(
        aimAndDriveCommand,
        Commands.waitSeconds(0.25).andThen(prepareShotCommand),
        Commands.waitSeconds(1)
            .andThen(Commands.runEnd(() -> indexer.set(-0.7), () -> indexer.set(0), indexer)));
  }

  public Command freeFire(Translation2d landMark) {
    final PrepareShotCommand shot =
        new PrepareShotCommand(shooter, () -> drive.getPose(), landMark);
    return Commands.sequence(
        Commands.run(() -> shot.execute(), shooter),
        Commands.waitUntil(() -> shot.isReadyToShoot()),
        Commands.run(() -> indexer.feed(), indexer));
  }

  public Command stop() {
    return Commands.run(() -> indexer.stopFeed(), indexer);
  }
}
