package frc.robot.subsystems.pixy2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Pixy2 extends SubsystemBase {
  private final Pixy2IO io;
  private final Pixy2IOInputsAutoLogged inputs = new Pixy2IOInputsAutoLogged();

  /** Used to share robot state across subsystems */
  private final RobotState robotState = RobotState.getInstance();

  public Pixy2(Pixy2IO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pixy2", inputs);

    // Currently we just share presence of a note and the angle to the note
    // TODO: Distance to the note is TBD
    robotState.seeNote = seesNote();
    robotState.noteAngle = getAngle();
  }

  public double getAge() {
    return inputs.age;
  }

  public boolean seesNote() {
    return inputs.seesNote;
  }

  public double getX() {
    return inputs.x;
  }

  public double getY() {
    return inputs.y;
  }

  public double getAngle() {
    return inputs.angle;
  }

  public double getSignature() {
    return inputs.signature;
  }

  public double getWidth() {
    return inputs.width;
  }

  public double getHeight() {
    return inputs.height;
  }
}
