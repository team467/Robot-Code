// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  enum IndexerState {
    FOWARD,
    HOLD,
    BACKWARD
  }

  IndexerState indxState = IndexerState.HOLD;

  /** Creates a new shooter. */
  private final ShooterIO io;

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    switch (indxState) {
      case FOWARD:
        io.setIndexerVoltage(ShooterConstants.indexerFowardVoltage);
        break;
      case HOLD:
        io.setIndexerVoltage(ShooterConstants.indexerHoldVoltage);
        break;
      case BACKWARD:
        io.setIndexerVoltage(ShooterConstants.indexerBackwardVolatage);
        break;
      default:
        break;
    }
  }

  public void setIndexeVoltage(double volts) {
    io.setIndexerVoltage(volts);
  }

  public void SetIndexerState(IndexerState indxState) {
    this.indxState = indxState;
}

public void setShooterVoltage(double volts) {
  io.setShooterVoltage(volts);
}

public boolean getShooterIsReady() {
  return inputs.shooterVelocityRadPerSec >= ShooterConstants.shooterReadyVelocity;
}

public boolean getHoldingNote() {
  return true;
}
}
