package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
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
    RobotState.getInstance().climberDeployed = inputs.climberDeployed;
    RobotState.getInstance().climberWinched = inputs.climberWinched;
  }
  /**
   * Command to winch the climber.
   *
   * @return the winch command.
   */
  public Command winch() {
    return Commands.run(() -> io.setSpeed(ClimberConstants.WINCH_SPEED), this)
        .until(() -> inputs.position <= ClimberConstants.WINCHED_POSITION);
  }

  public Command deploy() {
    return Commands.run(() -> io.setSpeed(ClimberConstants.DEPLOY_SPEED), this)
        .until(() -> inputs.climberDeployed);
  }

  public Command goForward() {
    return Commands.run(() -> io.setSpeed(0.80), this).until(() -> inputs.climberDeployed);
  }

  public Command goBackward() {
    return Commands.run(() -> io.setSpeed(-0.80), this);
  }

  public Command stop() {
    return Commands.run(() -> io.setSpeed(0), this);
  }
}
