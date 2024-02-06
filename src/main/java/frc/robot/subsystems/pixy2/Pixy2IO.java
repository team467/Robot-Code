package frc.robot.subsystems.pixy2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  NetworkTable pixyTable = networkTables.getTable("Pixy2");

  default void updateInputs(Pixy2IOInputs inputs) {}
}
