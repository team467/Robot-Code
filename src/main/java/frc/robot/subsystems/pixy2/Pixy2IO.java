package frc.robot.subsystems.pixy2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
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
  }

  final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  final NetworkTable pixyTable = networkTables.getTable("Pixy2");

  public default void initialize() {
    Commands.runOnce(
        () -> {
          networkTables.startClient3("Pixy2Reader");
        });
  }

  default void updateInputs(Pixy2IOInputs inputs) {}
}
