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
  /**
   * @param percent
   * @return A command that sets the indexer to a percent velocity from -1 to 1
   */
  public Command setIndexerPercentVoltage(double percent) {
    return Commands.run(() -> io.setIndexerPercentVelocity(percent), this);
  }
  /**
   * @return A command that sets the indexer voltage to zero
   */
  public Command stop() {
    return Commands.run(() -> io.setIndexerVoltage(0), this);
  }
  /**
   * @param volts
   * @return A command that sets the indexer voltage to the inputed volts
   */
  public Command setIndexerVoltage(double volts) {
    return Commands.run(() -> io.setIndexerVoltage(volts), this);
  }
  /**
   * @return if the indexers limit switch is pressed
   */
  public boolean getLimitSwitchPressed() {
    return inputs.indexerLimitSwitchPressed;
  }
}
