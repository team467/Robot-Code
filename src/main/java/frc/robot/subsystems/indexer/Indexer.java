package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.robotstate.RobotState;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final RobotState robotState;

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /** Creates a new Indexer. */
  public Indexer(IndexerIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    robotState.hasNote = getLimitSwitchPressed();
  }
  /**
   * @param percent
   * @return A command that sets the indexer to a percent velocity from -1 to 1
   */
  public Command setIndexerPercentVelocity(double percent) {
    return Commands.run(() -> io.setIndexerPercentVelocity(percent), this);
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
    return inputs.indexerLimitSwitchLeftPressed || inputs.indexerLimitSwitchRightPressed;
  }
}
