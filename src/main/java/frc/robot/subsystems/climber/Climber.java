package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
  /**
   * Command to winch the climber.
   *
   * @return the winch command.
   */
  public Command winch() {
    return Commands.run(() -> io.setSpeed(-1.0), this)
        .until(() -> inputs.position <= ClimberConstants.WINCHED_POSITION);
  }

  public Command deploy() {
    return Commands.run(() -> io.setSpeed(1.0), this)
        .until(() -> inputs.position <= ClimberConstants.DEPLOYED_POSITION);
  }
}
