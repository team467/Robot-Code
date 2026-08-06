package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.extend.IntakeExtend.State;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public enum State {
    IDLE,
    RUNNING,
    REVERSE
  }

  public State rollerState = State.IDLE;

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  /** Updates roller telemetry and exposes the active intake state to robot-wide logic. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/IntakeRollers", inputs);
    RobotState.getInstance().intaking = inputs.intakeVolts > 0;
    switch (rollerState) {
      case IDLE -> setVoltageIntake(0.0);
      case RUNNING -> setVoltageIntake(8.0);
      case REVERSE -> setVoltageIntake(-8.0);
    }
  }

  /** Drives the intake rollers with open-loop voltage. */
  public void setVoltageIntake(double intakeVolts) {
    io.setVoltageIntake(intakeVolts);
  }

  /** Runs rollers in reverse (outtake). */
  public Command outtake() {
    return runEnd(() -> rollerState = State.REVERSE, () -> rollerState = State.IDLE);
  }

  /** Stops rollers by setting state to IDLE. */
  public Command stopIntakeCommand() {
    return runOnce(() -> rollerState = State.IDLE);
  }

  /**
   * Returns whether the intake rollers are actively intaking.
   *
   * @return True if intaking (voltage is positive)
   */
  public boolean isIntaking() {
    return inputs.intakeVolts > 0.0;
  }

  // CAUTION: Do not use unless absolutely necessary, may cause unexpected behavior
  public void overrideState(State newState) {
    rollerState = newState;
  }
}
