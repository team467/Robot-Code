package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.KS.get(), ElevatorConstants.KG.get(), ElevatorConstants.KV.get());
  private ProfiledPIDController feedback =
      new ProfiledPIDController(
          ElevatorConstants.KP.get(),
          0,
          ElevatorConstants.KD.get(),
          new TrapezoidProfile.Constraints(
              ElevatorConstants.MAX_VELOCITY.get(), ElevatorConstants.MAX_ACCELERATION.get()));
  private boolean feedbackMode = true;

  @AutoLogOutput private boolean isCalibrated = false;
  @AutoLogOutput private boolean holdLock = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    io.updateInputs(inputs);

    feedback.disableContinuousInput();
    feedback.reset(inputs.positionMeters);
    feedback.setGoal(inputs.positionMeters);

    feedback.setTolerance(ElevatorConstants.TOLERANCE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Elevator", inputs);

    if (DriverStation.isDisabled()) {
      feedback.reset(inputs.positionMeters, inputs.velocityMetersPerSec);
    }

    if (Constants.tuningMode) {
      if (ElevatorConstants.KS.hasChanged(hashCode())
          || ElevatorConstants.KG.hasChanged(hashCode())
          || ElevatorConstants.KV.hasChanged(hashCode())) {
        feedforward =
            new ElevatorFeedforward(
                ElevatorConstants.KS.get(), ElevatorConstants.KG.get(), ElevatorConstants.KV.get());
      }

      if (ElevatorConstants.KP.hasChanged(hashCode())
          || ElevatorConstants.KD.hasChanged(hashCode())
          || ElevatorConstants.MAX_VELOCITY.hasChanged(hashCode())
          || ElevatorConstants.MAX_ACCELERATION.hasChanged(hashCode())) {

        feedback.setPID(ElevatorConstants.KP.get(), 0, ElevatorConstants.KD.get());
        feedback.setConstraints(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY.get(), ElevatorConstants.MAX_ACCELERATION.get()));
      }
    }

    if (inputs.limitSwitchPressed) {
      io.resetPosition();
      if (!isCalibrated) {
        feedback.reset(ElevatorConstants.STOW);
        isCalibrated = true;
      }
    }

    Logger.recordOutput("Elevator/PIDEnabled", feedbackMode);
    if (feedbackMode) {
      io.setVoltage(
          feedback.calculate(inputs.positionMeters)
              + feedforward.calculate(
                  feedback.getSetpoint().position, feedback.getSetpoint().velocity));

      Logger.recordOutput("Elevator/Goal/Position", feedback.getGoal().position);
      Logger.recordOutput("Elevator/Goal/Velocity", feedback.getGoal().velocity);
      Logger.recordOutput("Elevator/Setpoint/Position", feedback.getSetpoint().position);
      Logger.recordOutput("Elevator/Setpoint/Velocity", feedback.getSetpoint().velocity);
      Logger.recordOutput("Elevator/Measurement/Position", inputs.positionMeters);
    } else {
      feedback.reset(
          new TrapezoidProfile.State(inputs.positionMeters, inputs.velocityMetersPerSec));
    }
  }

  public Command toSetpoint(double setpointAngle) {
    return Commands.run(
        () -> {
          feedback.setGoal(setpointAngle);
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

  public Command hold() {
    return Commands.sequence(
        Commands.sequence(
                Commands.runOnce(() -> io.setVoltage(0.05 * 12), this), Commands.waitSeconds(0.5))
            .onlyIf(() -> inputs.velocityMetersPerSec < 0),
        Commands.run(
                () -> {
                  if (!holdLock) {
                    holdLock = true;
                    if (inputs.velocityMetersPerSec < 0) {
                      feedback.setGoal(inputs.positionMeters);
                    } else {
                      feedback.setGoal(inputs.positionMeters);
                    }
                    feedbackMode = true;
                  }
                },
                this)
            .finallyDo(() -> holdLock = false));
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public boolean atSetpoint() {
    return feedback.atGoal();
  }

  public boolean limitSwitchPressed() {
    return inputs.limitSwitchPressed;
  }
}
