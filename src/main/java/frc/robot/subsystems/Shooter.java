package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class Shooter extends SubsystemBase {
  private final AngularVelocity velocityTolerance = RPM.of(100);

  private final TalonFX shooter1 = new TalonFX(10);
  private final TalonFX shooter2 = new TalonFX(11);
  private final TalonFX shooter3 = new TalonFX(12);
  private final List<TalonFX> motors = List.of(shooter1, shooter2, shooter3);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final AngularVelocity freeSpeed = RPM.of(6000);

  public Shooter() {
    final TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12 / freeSpeed.in(RotationsPerSecond)));

    shooter1.getConfigurator().apply(config);
    shooter2.getConfigurator().apply(config);
    shooter3.getConfigurator().apply(config);

    shooter3.setControl(new Follower(10, MotorAlignmentValue.Aligned));
  }

  public void setRPM(double rpm) {
    shooter1.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
  }

  public void setPercent(double percent) {
    shooter2.set(percent);
    shooter3.set(-percent);
  }

  public void stop() {
    shooter1.setControl(velocityRequest.withVelocity(RPM.of(0)));
  }

  public void stopPercent() {
    shooter1.set(0);
  }

  public boolean isVelocityWithinTolerance() {
    return motors.stream()
        .allMatch(
            motor -> {
              final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
              final AngularVelocity currentVelocity = motor.getVelocity().getValue();
              final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
              return isInVelocityMode && currentVelocity.isNear(targetVelocity, velocityTolerance);
            });
  }
}
