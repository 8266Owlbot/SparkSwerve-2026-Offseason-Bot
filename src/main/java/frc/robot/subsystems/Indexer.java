package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX indexer1 = new TalonFX(13);
  private final TalonFX indexer2 = new TalonFX(14);

  public Indexer() {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true));

    indexer1.getConfigurator().apply(config);
    indexer2.getConfigurator().apply(config);

    indexer1.setControl(new Follower(13, MotorAlignmentValue.Opposed));
  }

  public void set(double set) {
    indexer1.set(set);
    indexer2.set(-set);
  }

  public Command feed() {
    return Commands.run(() -> set(1), this);
  }

  public Command stopFeed() {
    return Commands.run(() -> set(0), this);
  }
}
