package frc.robot.subsystems.pixy2;

import org.littletonrobotics.junction.AutoLog;

public interface Pixy2IO {
  void updateInputs(Pixy2IOInputs inputs);

  @AutoLog
  class Pixy2IOInputs {
    public double age = 0.0;
    public double x = 0.0;
    public double y = 0.0;
    public double angle = 0.0;
    public double signature = 0.0;
    public double width = 0.0;
    public double height = 0.0;
  }

  default void updateInputs(Pixy2IO inputs) {}
}
