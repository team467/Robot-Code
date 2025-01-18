package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;

  private NetworkTable climberTable;
  private NetworkTableEntry climberTestingEntry;

  private boolean autoMove = false;

  // Standard classes for controlling our climber
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ClimberConstants.CLIMBER_P,
          ClimberConstants.CLIMBER_I,
          ClimberConstants.CLIMBER_D,
          new TrapezoidProfile.Constraints(2.45, 2.45));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ClimberConstants.CLIMBER_S,
          ClimberConstants.CLIMBER_G,
          ClimberConstants.CLIMBER_V,
          ClimberConstants.CLIMBER_A);

  public Climber(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();

    climberTable = NetworkTableInstance.getDefault().getTable("Climber");
    climberTestingEntry = climberTable.getEntry("Height");
    climberTestingEntry.setDouble(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    if (DriverStation.isTest()) {
      autoMove = true;
      controller.setGoal(climberTestingEntry.getDouble(0.0));
    }

    if (autoMove) {
      // With the setpoint value we run PID control like normal
      double pidOutput = controller.calculate(inputs.position);
      double feedforwardOutput = feedforward.calculate(controller.getSetpoint().velocity);
      io.setVoltage(pidOutput + feedforwardOutput);
    }

    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    RobotState.getInstance().climberRatchet = inputs.ratchetLocked;

    if (getLimitSwitch()) {
      io.resetPosition();
    }
  }

  /**
   * Command to raise or lower the climber arms If percentOutput is negative, the climber will lower
   * If percentOutput is positive, the climber will raise If percentOutput is 0, the climber will
   * stop If the ratchet is locked, the climber will not move. It does nothing before the command
   * and then checks if the ratchet is locked. Ends the command if the ratchet is locked.
   *
   * @param percentOutput takes a number from -1 to 1.
   * @return no return
   */
  public Command raiseOrLower(double speed) {
    return Commands.run(
            () -> {
              Logger.recordOutput("Climber/OutputDesired", speed);
              io.setSpeed(speed);
            },
            this)
        .onlyWhile(() -> !inputs.ratchetLocked)
        .beforeStarting( // This is for testing the motors before. Might be taken out.
            Commands.none()
                .alongWith(
                    Commands.runOnce(
                        () -> {
                          RobotState.getInstance().climberUp = speed > 0;
                          RobotState.getInstance().climberDown = speed < 0;
                        })))
        .onlyWhile(() -> !inputs.ratchetLocked)
        .finallyDo(
            () -> {
              RobotState.getInstance().climberUp = false;
              RobotState.getInstance().climberDown = false;
            });
  }

  /**
   * Command to disable the climber
   *
   * @return no return
   */
  public Command setRatchet(boolean locked) {
    return Commands.runOnce(
        () -> {
          io.setRatchetLocked(locked);
        },
        this);
  }

  // TODO: Review design to see if we need to move backwards to release ratchet.

  public boolean getRatchet() {
    return inputs.ratchetLocked;
  }

  public boolean getLimitSwitch() {
    return true;
  }
}
