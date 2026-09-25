package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class Landmarks {
  // public static final Pose2d inicioXNeutralZone = new ;
  public static final double MitadYNeutralZone = 158.32;

  public static Translation2d hubPosition() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return new Translation2d(Inches.of(182.105), Inches.of(158.845));
    }
    return new Translation2d(Inches.of(469.115), Inches.of(158.845));
  }

  public static Translation2d allianceLeftZone() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return new Translation2d(Inches.of(27.0), Inches.of(240.71));
    }
    return new Translation2d(Inches.of(623.12), Inches.of(75.96));
  }

  public static Translation2d allianceRightZone() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return new Translation2d(Inches.of(27.0), Inches.of(75.96));
    }
    return new Translation2d(Inches.of(623.12), Inches.of(240.71));
  }

  public static Translation2d neutralLeftZone() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      return new Translation2d(Inches.of(27.0), Inches.of(75.96));
    }
    return new Translation2d(Inches.of(623.12), Inches.of(240.71));
  }
}
