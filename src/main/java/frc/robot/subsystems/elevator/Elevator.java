package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  @AutoLogOutput private boolean isManual = false;
  private double manualPercent = 0.0;
  @AutoLogOutput private boolean holdLock = false;
  @AutoLogOutput private double holdPosition = INTAKE_POSITION;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    io.updateInputs(inputs);
    holdPosition = inputs.positionMeters;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    if (isManual) {
      io.setPercent(manualPercent);
    } else {
      io.goToSetpoint();
    }
  }

  /**
   * Checks if the elevator has reached its setpoint.
   *
   * @return true if the elevator is at the target setpoint, false otherwise.
   */
  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  /**
   * Creates a command to move the elevator to the specified position. If the elevator is not
   * calibrated, it will first run the calibration routine before moving to the setpoint.
   *
   * @param setpointMeters The desired position in meters.
   * @return A command that moves the elevator to the setpoint. If the elevator is not calibrated,
   *     it will first calibrate and then move to the setpoint.
   */
  public Command toSetpoint(double setpointMeters) {
    return Commands.run(
        () -> {
          this.isManual = false;
          io.setPosition(setpointMeters);
        },
        this);
  }

  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          this.isManual = true;
          this.manualPercent = percent;
        },
        this);
  }

  /*   public Command setHoldPosition(double holdPosition) {
      return Commands.runOnce(
          () -> {
            this.holdPosition = holdPosition;
          },
          this);
    }

  /*   public Command hold() {
      return Commands.runOnce(() -> holdPosition = inputs.positionMeters, this)
          .alongWith(
              Commands.run(
                  () -> {
                    io.hold(holdPosition);
                  },
                  this));
    } */

  public double getPosition() {
    return inputs.positionMeters;
  }

  public boolean limitSwitchPressed() {
    return inputs.stowLimitSwitch;
  }
}
