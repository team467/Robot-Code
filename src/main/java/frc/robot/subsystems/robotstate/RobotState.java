package frc.robot.subsystems.robotstate;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {

  private static RobotState instance = null;

  private final RobotStateIO io;
  private final RobotStateIOInputsAutoLogged inputs = new RobotStateIOInputsAutoLogged();

  private RobotState(RobotStateIO io) {
    this.io = io;
    this.io.initialize(inputs);
    this.io.updateInputs(inputs);
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState(new RobotStateIOPhysical());
    }
    return instance;
  }

  public RobotStateIOInputsAutoLogged state() {
    return inputs;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("RobotState", inputs);
  }
}
