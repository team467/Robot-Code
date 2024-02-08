package frc.robot.subsystems.pixy2;

import org.littletonrobotics.junction.AutoLog;

public interface Pixy2IO {

  @AutoLog
  class Pixy2IOInputs {
    public double age;
    public double x;
    public double y;
    public double angle;
    public double signature;
    public double width;
    public double height;

    public boolean seesNote;
  }

  default void updateInputs(Pixy2IOInputs inputs) {}
}
