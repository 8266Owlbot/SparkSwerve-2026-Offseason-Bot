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
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX indexer1 = new TalonFX(13);
  private final TalonFX indexer2 = new TalonFX(14);

  private final SparkMax floor1 = new SparkMax(15, MotorType.kBrushless);
  private final SparkMax floor2 = new SparkMax(17, MotorType.kBrushless);

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

    final SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(20);

    indexer1.getConfigurator().apply(config);
    indexer2.getConfigurator().apply(config);

    floor1.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    floor2.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexer2.setControl(new Follower(13, MotorAlignmentValue.Opposed));
  }

  public void set(double set) {
    indexer1.set(set);
    floor1.set(-set * 0.5);
    floor2.set(set * 0.5);
  }

  public Command feed() {
    return Commands.run(() -> set(1), this);
  }

  public Command stopFeed() {
    return Commands.run(() -> set(0), this);
  }
}
