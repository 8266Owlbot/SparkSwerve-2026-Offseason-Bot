package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class SubsystemCommands extends Command {
  private final Drive drive;
  private final DoubleSupplier fordwardInput;
  private final DoubleSupplier leftInput;

  public SubsystemCommands(Drive drive, DoubleSupplier fordwardInput, DoubleSupplier leftInput) {
    this.drive = drive;
    this.fordwardInput = fordwardInput;
    this.leftInput = leftInput;
  }

  public Command aimAndShoot() {
    final AimAndDriveCommand aimAndDriveCommand =
        new AimAndDriveCommand(drive, fordwardInput, leftInput);
    return Commands.parallel(aimAndDriveCommand);
  }
}
