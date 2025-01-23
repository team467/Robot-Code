package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

 

  /** Moves algae arm inwards. */
  private Command retractArm() {
    return new FunctionalCommand(
        () -> {
          io.setPivotVolts(AlgaeEffectorConstants.RETRACT_VOLTAGE);
          armExtended = false;
        },
        () -> {
        },
        interrupted -> io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOLTAGE),
        () -> inputs.reverseLimitSwitch,
        this)
        .withTimeout(0.5);
  }
  
 /** Moves algae arm outwards */
  private Command extendArm() {
    return new FunctionalCommand(
            () -> {
              io.setPivotVolts(AlgaeEffectorConstants.EXTEND_VOLTAGE);
              armExtended = true;
          },
            () -> {

            },
        interrupted -> io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOLTAGE),
            () -> inputs.forwardLimitSwitch,    
            this)
        .withTimeout(0.5);
  }

  /** Makes arm either go in or out */
  public Command toggleArm() {
    return Commands.either(retractArm(), extendArm(), () -> armExtended);
  }

  public Command startAlgaeRemoval() {
    return Commands.either(stopRemoval(), startRemoval(), () -> algaeMotorStarted);
  }

  /** Spins the motor to start the removal of algae */
  private Command startRemoval() {
    return Commands.startEnd(
        () -> io.setRemovalVolts(AlgaeEffectorConstants.REMOVAL_VOLTAGE),
        () -> io.setRemovalVolts(AlgaeEffectorConstants.ZERO_VOLTAGE),
        this);
  }

  /** Stops spinning the removal motor */
  private Command stopRemoval() {
    return this.runOnce(() -> io.setRemovalVolts(AlgaeEffectorConstants.ZERO_VOLTAGE));
  }
  /** When the arm is extended, it starts the algae motor too */
  public Command extendSpinAlgae() {
    return Commands.sequence(extendArm(), startRemoval());
  }
  /** When the arm is retracted, the motor automatically stops */
  public Command retractStopAlgae() {
    return Commands.sequence(stopRemoval(), retractArm());
  }
}
