package frc.robot.subsystems.pixy2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pixy2 extends SubsystemBase {
  private final Pixy2IO io;
  private final Pixy2IOInputsAutoLogged inputs = new Pixy2IOInputsAutoLogged();

  public Pixy2(Pixy2IO io) {
    this.io = io;
    io.initialize();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pixy2", inputs);
  }

  public double getAge() {
    return inputs.age;
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
