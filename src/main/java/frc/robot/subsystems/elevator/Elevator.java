package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private boolean autoMove = false;

  // Standard classes for controlling our elevator
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.ELEVATOR_P,
          Constants.ELEVATOR_I,
          Constants.ELEVATOR_D,
          new TrapezoidProfile.Constraints(2.45, 2.45));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          Constants.ELEVATOR_S, Constants.ELEVATOR_G, Constants.ELEVATOR_V, Constants.ELEVATOR_A);

  public Elevator(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (autoMove) {
      // With the setpoint value we run PID control like normal
      double pidOutput = controller.calculate(inputs.position);
      double feedforwardOutput = feedforward.calculate(controller.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);
    }
  }

  public Command stop() {
    return Commands.run(
        () -> {
          Logger.recordOutput("Elevator/DesiredSpeed", 0);
          this.autoMove = false;
          io.setVoltage(0);
        },
        this);
  }

  public Command goToPosition(double position) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Elevator/DesiredPosition", position);
          this.autoMove = true;
          controller.setGoal(position);
        },
        this);
  }

  public Command manual(double speed) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Elevator/DesiredSpeed", speed);
          this.autoMove = false;
          io.setVoltage(speed * RobotController.getBatteryVoltage());
        },
        this);
  }
}
