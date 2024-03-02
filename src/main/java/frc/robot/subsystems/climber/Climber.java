package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberIOInputs = new ClimberIOInputsAutoLogged();
  private PIDController climberLeftFeedback =
      new PIDController(ClimberConstants.CLIMBER_KP.get(), 0, ClimberConstants.CLIMBER_KD.get());
  private PIDController climberRightFeedback =
      new PIDController(ClimberConstants.CLIMBER_KP.get(), 0, ClimberConstants.CLIMBER_KD.get());
  private boolean PIDMode = false;
  private double setpointLeft;
  private double setpointRight;

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
    if (PIDMode) {
      climberIO.setLeftMotorVolts(
          climberLeftFeedback.calculate(climberIOInputs.ClimberLeftPosition, setpointLeft));
      climberIO.setRightMotorVolts(
          climberRightFeedback.calculate(climberIOInputs.ClimberRightPosition, setpointRight));
    }
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
          PIDMode = false;
          climberIO.setMotorsOutputPercent(percentOutput, percentOutput > 0);
        },
        this);
  }

  public Command raiseToSetpoint(double setpointLeft, double setPointRight) {
    return Commands.run(
        () -> {
          this.setpointLeft = setpointLeft;
          this.setpointRight = setPointRight;
          PIDMode = true;
          climberIO.setRatchetLocked(false);
        },
        this);
  }

  /**
   * Command to disable the climber
   *
   * @return no return
   */
  public Command stop() {
    return Commands.run(
        () -> {
          PIDMode = false;
          climberIO.setMotorsOutputPercent(0, false);
          climberIO.setRatchetLocked(true);
        },
        this);
  }
}
