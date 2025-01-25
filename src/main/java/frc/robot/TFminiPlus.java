package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;

public class TFminiPlus {

  private final SerialPort serialPort;
  private final NetworkTable networkTable;

  public TFminiPlus(SerialPort.Port port, String tableName) {
    serialPort =
        new SerialPort(
            115200, port); // I am using 115200 becaus is the default baud rate, could be changed
    networkTable = NetworkTableInstance.getDefault().getTable(tableName);
  }

  // Reads and parses data from the sensor
  public void updateSensorData() {
    byte[] data = serialPort.read(9); // The data frame is 9 bytes long

    if (data.length == 9 && isVaildFrame(data)) {
      int distance = parseDistance(data);
      int strength = parseStrength(data);

      // Sets data to NetworkTables
      networkTable.getEntry("Distance").setDouble(distance);
      networkTable.getEntry("Strength").setDouble(strength);
    }
  }

  // Checks if the data frame is valid
  private boolean isVaildFrame(byte[] data) {
    // Start bytes must be 0x59 and 0x59
    return (data[0] & 0xFF) == 0x59 && (data[1] & 0xFF) == 0x59;
  }

  // Parses the distance value from the data frame
  private int parseDistance(byte[] data) {
    // Distance is in bytes 2 and 3
    return (data[2] & 0xFF) | ((data[3] & 0xFF) << 8);
  }

  // Parses the signal strength value from the data frame
  private int parseStrength(byte[] data) {
    // Strength is in bytes 4 and 5
    return (data[4] & 0xFF) | ((data[5] & 0xFF) << 8);
  }

  public void periodic() {
    updateSensorData();
  }
}
