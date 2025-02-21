package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  @AutoLogOutput private boolean isCalibrated = false;
  @AutoLogOutput private boolean holdLock = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    io.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (DriverStation.isDisabled()) {
      io.setPosition(inputs.positionMeters);
    }

    if (inputs.stowLimitSwitch) {
      io.resetPosition(elevatorToGround);
      if (!isCalibrated) {
        io.setPosition(STOW);
        isCalibrated = true;
      }
    }

    //    if (inputs.positionMeters > maxElevatorExtension && inputs.velocityMetersPerSec > 0) {
    //      io.setPosition(maxElevatorExtension);
    //    }
  }

  public Command toSetpoint(double setpointMeters) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Elevator/Setpoint", setpointMeters);
          io.setPosition(setpointMeters);
        },
        this);
  }

  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Elevator/DesiredVolts", percent * 12);
          io.setPercent(percent);
        },
        this);
  }

  public Command hold(double holdPosition) {
    return Commands.run(
        () -> {
          io.hold(holdPosition);
        },
        this);
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public boolean limitSwitchPressed() {
    return inputs.stowLimitSwitch;
  }
}
