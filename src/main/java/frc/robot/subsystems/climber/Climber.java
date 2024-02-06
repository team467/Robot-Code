package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();
  // private final Relay climberLock = new Relay(); //To do add Soloenoid

  public Climber(ClimberIO climberIO) {
    super();

    this.climberIO = climberIO;

    climberIO.updateInput(climberIOInputs);
  }

  public void raise() {}

  public void lower() {}

  public void stop() {
    climberIO.setLeftMotorOutputPercent(0);
    climberIO.setRightMotorOutputPercent(0);
  }

  public void disable() {
    climberIO.setLeftMotorOutputPercent(0);
    climberIO.setRightMotorOutputPercent(0);
  }
}
