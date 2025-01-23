package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class CoralEffector extends SubsystemBase {

  private CoralEffectorIO io;
  private EffectorIOInputsAutoLogged inputs = new EffectorIOInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance();
  private boolean PIDMode = false;
  private double currentVelocitySetpoint;

  public CoralEffector(CoralEffectorIO io) {
    this.io = io;
    this.inputs = new EffectorIOInputsAutoLogged();
  }

  public void Periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralEffector", inputs);
  }

  public boolean haveCoral() {
    return inputs.haveCoral;
  }

  public boolean coralOnTheWay() {
    return inputs.haveCoral;
  }

  public Command dumpCoral() {
    return Commands.run(
            () -> {
              io.setSpeed(CoralEffectorConstants.CORAL_EFFECTOR_SPEED_OUT.get());
            },
            this)
        .until(this::haveCoral);
  }

  public Command intakeCoral() {
    return Commands.run(
            () -> {
              io.setSpeed(CoralEffectorConstants.CORAL_INTAKE_SPEED.get());
            },
            this)
        .until(this::haveCoral);
  }

  // NEED TO ADD TEMPERATURE and OTHERS; Fix problems of getting using Limit Switch that is not in

}
