package first.robot.mechanisms.Intake.Rollers;

import first.robot.mechanisms.Intake.Rollers.IntakeRollerIO.IntakeRollerIOOutputs;
import first.robot.mechanisms.Intake.Rollers.IntakeRollerIO.RollerIOOutputMode;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

public class IntakeRoller implements Mechanism {
  private static final double INTAKE_VOLTS = 6.0;
  private static final double OUTTAKE_VOLTS = -6.0;

  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();
  private final IntakeRollerIOOutputs outputs = new IntakeRollerIOOutputs();

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(this::updateInputs);
  }

  private void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);
  }

  public void applyOutputs() {
    io.applyOutputs(outputs);
  }

  private void runVoltage(double volts) {
    outputs.mode = RollerIOOutputMode.VOLTAGE;
    outputs.targetVolts = volts;
  }

  private void stopOutput() {
    outputs.mode = RollerIOOutputMode.STOP;
  }

  public Command stop() {
    return run(coroutine -> {
          stopOutput();
          coroutine.park();
        })
        .named("Intake.Roller.Stop");
  }

  @Override
  public Command idle() {
    return run(coroutine -> {
          stopOutput();
          coroutine.park();
        })
        .named("Intake roller idle");
  }

  public Command intake() {
    return run(coroutine -> {
          runVoltage(INTAKE_VOLTS);
          coroutine.park();
        })
        .named("Intake");
  }

  public boolean isIntaking() {
    return outputs.mode == RollerIOOutputMode.VOLTAGE && outputs.targetVolts > 0;
  }

  public Command outtake() {
    return run(coroutine -> {
          runVoltage(OUTTAKE_VOLTS);
          coroutine.park();
        })
        .named("Outtake");
  }
}
