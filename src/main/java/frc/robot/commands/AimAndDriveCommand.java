package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AimAndDriveCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final PIDController thetaController;
  private boolean isAllign = false;
  private Translation2d shootGoal;
  private Optional<Alliance> alliance = DriverStation.getAlliance();

  public AimAndDriveCommand(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Translation2d shootGoal) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.shootGoal = shootGoal;

    this.thetaController = new PIDController(8, 0, 0);
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.thetaController.setTolerance(Units.degreesToRadians(3.0));

    addRequirements(drive);
  }

  public boolean isAimed() {
    return isAllign;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    Rotation2d desiredAngle = shootGoal.minus(robotPose.getTranslation()).getAngle();

    double velocityRotation =
        thetaController.calculate(robotPose.getRotation().getRadians(), desiredAngle.getRadians());

    isAllign = thetaController.atSetpoint();

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSupplier.getAsDouble(),
              ySupplier.getAsDouble(),
              velocityRotation,
              robotPose.getRotation()));
    } else {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              -xSupplier.getAsDouble(),
              -ySupplier.getAsDouble(),
              velocityRotation,
              robotPose.getRotation()));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
