package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class PrepareShotCommand extends Command {
  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  Interpolator.forDouble()
                      .interpolate(startValue.shooterRPM, endValue.shooterRPM, t)));

  static {
    distanceToShotMap.put(Inches.of(100), new Shot(5000));
    distanceToShotMap.put(Inches.of(200), new Shot(6000));
    distanceToShotMap.put(Inches.of(0), new Shot(0));
  }

  private final Shooter shooter;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Translation2d landmarks;

  public PrepareShotCommand(
      Shooter shooter, Supplier<Pose2d> robotPoseSupplier, Translation2d landmarks) {
    this.shooter = shooter;
    this.robotPoseSupplier = robotPoseSupplier;
    this.landmarks = landmarks;
    addRequirements(shooter);
  }

  public boolean isReadyToShoot() {
    return shooter.isVelocityWithinTolerance();
  }

  private Distance distanceToObjective() {
    final Translation2d robotPosition = robotPoseSupplier.get().getTranslation();
    final Translation2d objectivePosition = landmarks;
    return Meters.of(robotPosition.getDistance(objectivePosition));
  }

  @Override
  public void execute() {
    final Distance distanceToLandmark = distanceToObjective();
    final Shot shot = distanceToShotMap.get(distanceToLandmark);
    shooter.setRPM(shot.shooterRPM);
    SmartDashboard.putNumber("Distance to objective: ", distanceToLandmark.in(Inches));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  public static class Shot {
    public final double shooterRPM;

    public Shot(double shooterRPM) {
      this.shooterRPM = shooterRPM;
    }
  }
}
