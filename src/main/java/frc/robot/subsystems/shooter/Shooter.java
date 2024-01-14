// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  /** Creates a new shooter. */
  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setIndexerVoltage(double volts) {
    io.setIndexerVoltage(volts);
  }

  public void setShooterVoltage(double volts) {
    io.setShooterVoltage(volts);
  }

  public boolean getFlywheelSpeedIsReady() {
    return inputs.shooterVelocityRadPerSec >= ShooterConstants.shooterReadyVelocityRadPerSec;
  }

  public boolean getHoldingNote() {
    return true;
  }
}
