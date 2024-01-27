package frc.robot.subsystems.pixy2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Pixy2IOPhysical implements Pixy2IO {
  private final NetworkTableInstance networkTables = NetworkTableInstance.getDefault();

  private final NetworkTable pixyTable = networkTables.getTable("pixy2");

  @Override
  public void updateInputs(Pixy2IOInputs inputs) {
    inputs.age = pixyTable.getEntry("age").getDouble(0.0);
    inputs.x = pixyTable.getEntry("x").getDouble(0.0);
    inputs.y = pixyTable.getEntry("y").getDouble(0.0);
    inputs.angle = pixyTable.getEntry("angle").getDouble(0.0);
    inputs.signature = pixyTable.getEntry("signature").getDouble(0.0);
    inputs.width = pixyTable.getEntry("width").getDouble(0.0);
    inputs.height = pixyTable.getEntry("height").getDouble(0.0);
  }
}
