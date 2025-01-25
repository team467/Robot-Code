package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeEffector extends SubsystemBase {
  private boolean armExtended = false;
  private boolean algaeMotorStarted = false;
  private final AlgaeEffectorIO io;
  private PIDController pivotFeedback =
      new PIDController(AlgaeEffectorConstants.PIVOT_KP, 0, AlgaeEffectorConstants.PIVOT_KD);

  private final AlgaeEffectorIOInputsAutoLogged inputs = new AlgaeEffectorIOInputsAutoLogged();

  public AlgaeEffector(AlgaeEffectorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public Command stowArm() {
    return Commands.run(
            () -> {
              io.setPivotVolts(AlgaeEffectorConstants.RETRACT_VOLTAGE);
              io.setRemovalVolts(0);
            })
        .until(() -> inputs.isStowed);
  }

  /** When the arm is extended, it starts the algae motor too */
  public Command removeAlgae() {
    return Commands.run(
        () -> {
          io.setPivotVolts(AlgaeEffectorConstants.EXTEND_VOLTAGE);
          io.setRemovalVolts(AlgaeEffectorConstants.REMOVAL_VOLTAGE);
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setPivotVolts(0);
          io.setRemovalVolts(0);
        },
        this);
  }
}
