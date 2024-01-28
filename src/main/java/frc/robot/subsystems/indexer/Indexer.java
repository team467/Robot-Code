// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public Command setIndexerPercentVoltage(double percent) {
    return Commands.run(
        () -> io.setIndexerVoltage(IndexerConstants.INDEXER_MAX_VOLTAGE * percent), this);
  }

  public Command stop() {
    return Commands.run(() -> io.setIndexerVoltage(0), this);
  }

  public Command setIndexerVoltage(double volts) {
    return Commands.run(() -> io.setIndexerVoltage(volts), this);
  }

  public boolean getLimitSwitchPressed() {
    return inputs.indexerLimitSwitchPressed;
  }

  public String getIO() {
    return io.toString();
  }
}
