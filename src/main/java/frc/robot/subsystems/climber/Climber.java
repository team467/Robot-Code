package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();

  /**
   * ClimberIO object, gets inputs for ClimberIO object
   *
   * @param climberIO
   */
  public Climber(ClimberIO climberIO) {
    super();

    this.climberIO = climberIO;
  }

  public void periodic() {
    climberIO.updateInputs(climberIOInputs);
    Logger.processInputs("Climber", climberIOInputs);
    Logger.recordOutput("Climber/LeftLimitSwitch", climberIO.getLimitSwitchLeft());
    Logger.recordOutput("Climber/RightLimitSwitch", climberIO.getLimitSwitchRight());
  }

  /**
   * Command to raise or lower the climber arms If percentOutput is negative, the climber will lower
   * If percentOutput is positive, the climber will raise
   *
   * @param percentOutput takes a number from -1 to 1.
   * @return no return
   */
  public Command raiseOrLower(double percentOutput) {
    return Commands.run(
        () -> {
          climberIO.setRatchetLocked(false);
          climberIO.setMotorsOutputPercent(percentOutput);
        },
        this);
  }

  /**
   * Command to disable the climber
   *
   * @return no return
   */
  public Command unlockRatchet() {
    return Commands.run(
        () -> {
          climberIO.setRatchetLocked(false);
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          climberIO.setMotorsOutputPercent(0);
          climberIO.setRatchetLocked(true);
        },
        this);
  }
}
