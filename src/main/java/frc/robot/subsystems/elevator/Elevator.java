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
  private boolean feedbackMode = true;

  @AutoLogOutput private boolean isCalibrated = false;
  @AutoLogOutput private boolean holdLock = false;
  @AutoLogOutput private double setpoint;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    io.updateInputs(inputs);

    setpoint = inputs.positionMeters;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (DriverStation.isDisabled()) {
      setpoint = inputs.positionMeters;
    }

    if (inputs.limitSwitchPressed) {
      io.resetPosition(elevatorToGround);
      if (!isCalibrated) {
        setpoint = ElevatorConstants.STOW;
        isCalibrated = true;
      }
    }

    //    if (inputs.positionMeters > maxElevatorExtension && inputs.velocityMetersPerSec > 0) {
    //      io.setPosition(maxElevatorExtension);
    //    }

    Logger.recordOutput("Elevator/PIDEnabled", feedbackMode);
    if (feedbackMode) {
      io.setPosition(inputs.positionMeters);
    } else {
      setpoint = inputs.positionMeters;
    }
  }

  public Command toSetpoint(double setpointAngle) {
    return Commands.run(
        () -> {
          setpoint = setpointAngle;
          feedbackMode = true;
        },
        this);
  }

  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Elevator/DesiredVolts", percent * 12);
          io.setPercent(percent);
          feedbackMode = false;
        },
        this);
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public boolean atSetpoint() {
    return Math.abs(setpoint - inputs.positionMeters) < 0.05;
  }

  public boolean limitSwitchPressed() {
    return inputs.limitSwitchPressed;
  }
}
