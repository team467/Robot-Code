package frc.robot.subsystems.pixy2;

public class Pixy2IOPhysical implements Pixy2IO {

  public Pixy2IOPhysical() {
    networkTables.startClient4("RoboRio");
    networkTables.setServer("localhost");
    if (!networkTables.isConnected()) {
      System.out.println("Pixy2 NetworkTables not connected");
    } else {
      System.out.println("Pixy2 NetworkTables Connected");
    }
  }

  @Override
  public void updateInputs(Pixy2IOInputs inputs) {
    inputs.age = pixyTable.getEntry("Age").getDouble(0.0);
    inputs.x = pixyTable.getEntry("X").getDouble(0.0);
    inputs.y = pixyTable.getEntry("Y").getDouble(0.0);
    inputs.angle = pixyTable.getEntry("AngleDeg").getDouble(0.0);
    inputs.signature = pixyTable.getEntry("Signature").getDouble(0.0);
    inputs.width = pixyTable.getEntry("Width").getDouble(0.0);
    inputs.height = pixyTable.getEntry("Height").getDouble(0.0);
    inputs.seesNote = pixyTable.getEntry("SeesNote").getBoolean(false);
  }
}
