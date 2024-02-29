package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.robotstate.RobotState;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final RobotState robotState = RobotState.getInstance();

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    robotState.hasNote = getLimitSwitchPressed();
  }
  /**
   * @param percent from -1 to 1
   * @return A command that sets the indexer to a percent velocity from -1 to 1
   */
  public Command setPercent(double percent) {
    return Commands.run(() -> io.setIndexerPercentVelocity(percent), this);
  }
  /**
   * @param volts the voltage to set the indexer to
   * @return A command that sets the indexer voltage to the inputed volts
   */
  public Command setVolts(double volts) {
    return Commands.run(() -> io.setIndexerVoltage(volts), this);
  }
  /**
   * @return if the indexers limit switch is pressed
   */
  public boolean getLimitSwitchPressed() {
    return inputs.indexerLimitSwitchPressed;
  }
}
