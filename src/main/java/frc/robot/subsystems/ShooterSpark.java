// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSpark extends SubsystemBase {
  private SparkMax shooterIzq = new SparkMax(17, MotorType.kBrushless);
  private SparkMax shooterDer = new SparkMax(16, MotorType.kBrushless);

  /** Creates a new ShooterSpark. */
  public ShooterSpark() {
    var config1 = new SparkMaxConfig();
    config1.inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(40);

    shooterIzq.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterDer.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPercent(double percent) {
    shooterIzq.set(percent);
    shooterDer.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double izq = shooterIzq.getAppliedOutput();
    SmartDashboard.putNumber("Aplied Output Shooter Izq", izq);

    double der = shooterDer.getAppliedOutput();
    SmartDashboard.putNumber("Aplied Output Shooter Der", der);
  }
}
