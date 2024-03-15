package frc.robot.subsystems.pixy2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Pixy2IOPhysical implements Pixy2IO {
  NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
  NetworkTable pixyTable = networkTables.getTable("Pixy2");

  private int age = 0;

  public Pixy2IOPhysical() {
    networkTables.startClient4("RoboRio");
    networkTables.setServer("localhost");
    if (!networkTables.isConnected()) {
      System.out.println("Pixy2 NetworkTables not connected");
    } else {
      System.out.println("Pixy2 NetworkTables Connected");
    }
  }

  public boolean hasChanged(Pixy2IOInputs inputs) {
    if (inputs.x != pixyTable.getEntry("X").getDouble(0.0)) {
      return true;
    }
    if (inputs.y != pixyTable.getEntry("Y").getDouble(0.0)) {
      return true;
    }
    return false;
  }

  @Override
  public void updateInputs(Pixy2IOInputs inputs) {
    if (hasChanged(inputs)) {
      age = 0;
    } else {
      age++;
    }
    inputs.age = pixyTable.getEntry("Age").getDouble(0.0);
    inputs.x = pixyTable.getEntry("X").getDouble(0.0);
    inputs.y = pixyTable.getEntry("Y").getDouble(0.0);
    inputs.angle = pixyTable.getEntry("AngleDeg").getDouble(0.0);
    inputs.signature = pixyTable.getEntry("Signature").getDouble(0.0);
    inputs.width = pixyTable.getEntry("Width").getDouble(0.0);
    inputs.height = pixyTable.getEntry("Height").getDouble(0.0);
    inputs.seesNote = pixyTable.getEntry("Valid").getBoolean(false) && age < 1000;
  }
}
