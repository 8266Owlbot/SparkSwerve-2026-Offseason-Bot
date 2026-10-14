package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final AngularVelocity velocityTolerance = RPM.of(200);

  private final TalonFX shooter1 = new TalonFX(10);
  private final TalonFX shooter2 = new TalonFX(11);
  private final TalonFX shooter3 = new TalonFX(12);

  StatusSignal<AngularVelocity> velocitySignal = shooter2.getVelocity();

  private final List<TalonFX> motors = List.of(shooter2, shooter3);
  private final SysIdRoutine sysIdRoutine;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut sysIdControl = new VoltageOut(0);
  private final AngularVelocity freeSpeed = RPM.of(6000);

  public Shooter() {
    final TalonFXConfiguration talonConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withVoltage(new VoltageConfigs().withPeakReverseVoltage(Volts.of(0)))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.2)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKS(0.3)
                    .withKV(12 / freeSpeed.in(RotationsPerSecond))
                    .withKA(0));

    shooter1.getConfigurator().apply(talonConfig);
    shooter2.getConfigurator().apply(talonConfig);
    shooter3.getConfigurator().apply(talonConfig);

    shooter2.setControl(new Follower(10, MotorAlignmentValue.Aligned));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> shooter1.setControl(sysIdControl.withOutput(voltage.in(Volts))),
                (log) -> {
                  log.motor("shooter")
                      .voltage(Volts.of(shooter1.getMotorVoltage().getValueAsDouble()))
                      .angularPosition(Rotations.of(shooter1.getPosition().getValueAsDouble()))
                      .angularVelocity(
                          RotationsPerSecond.of(shooter1.getVelocity().getValueAsDouble()));
                },
                this));
  }

  public void setRPM(double rpm) {
    shooter1.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    Logger.recordOutput("Shooter RPM Commanded", rpm);
  }

  public void setPercent(double percent) {
    shooter1.setControl(velocityRequest.withAcceleration(percent));
  }

  public void stop() {
    shooter1.setControl(velocityRequest.withVelocity(RPM.of(0)));
  }

  public void stopPercent() {
    shooter1.setControl(velocityRequest.withAcceleration(0));
  }

  public boolean isVelocityWithinTolerance() {
    final boolean isMasterInVelocityMode = shooter1.getAppliedControl().equals(velocityRequest);
    final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();

    return isMasterInVelocityMode
        && motors.stream()
            .allMatch(
                motor -> {
                  final AngularVelocity currentVelocity = motor.getVelocity().getValue();
                  return currentVelocity.isNear(targetVelocity, velocityTolerance);
                });
  }

  @Override
  public void periodic() {
    velocitySignal.refresh();
    double rpmShooter = velocitySignal.getValueAsDouble() * 60.0;
    Logger.recordOutput("Shooter RPM Measured", rpmShooter);
  }

  public Command sysIdQuasystic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
