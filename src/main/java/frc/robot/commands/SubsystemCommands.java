package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class SubsystemCommands extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final DoubleSupplier fordwardInput;
  private final DoubleSupplier leftInput;

  public SubsystemCommands(
      Drive drive, Shooter shooter, DoubleSupplier fordwardInput, DoubleSupplier leftInput) {
    this.drive = drive;
    this.shooter = shooter;
    this.fordwardInput = fordwardInput;
    this.leftInput = leftInput;
  }

  public Command aimAndShoot(Translation2d landMark) {
    final AimAndDriveCommand aimAndDriveCommand =
        new AimAndDriveCommand(drive, fordwardInput, leftInput, landMark);
    final PrepareShotCommand prepareShotCommand =
        new PrepareShotCommand(shooter, () -> drive.getPose(), landMark);
    return Commands.parallel(
        aimAndDriveCommand, Commands.waitSeconds(0.25).andThen(prepareShotCommand));
  }
}
