package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;

public class TFminiPlus {

  private final SerialPort serialPort;
  private final NetworkTable networkTable;

  private int distance = 0;
  private int strength = 0;
  private double temperature = 0.0;

  public TFminiPlus(SerialPort.Port port, String TFminiPlus) {
    serialPort =
        new SerialPort(
            115200, port); // I am using 115200 because is the default baud rate, could be changed
    networkTable = NetworkTableInstance.getDefault().getTable(TFminiPlus);
  }

  // Reads and parses data from the sensor
  public void updateSensorData() {
    byte[] data = serialPort.read(9);

    if (data.length == 9 && isVaildFrame(data)) {
      distance = parseDistance(data);
      strength = parseStrength(data);
      temperature = parseTemperature(data);

      networkTable.getEntry("Distance").setDouble(distance);
      networkTable.getEntry("Strength").setDouble(strength);
      networkTable.getEntry("Temperature").setDouble(temperature);
    }
  }

  private boolean isVaildFrame(byte[] data) {
    // Start bytes must be 0x59 and 0x59
    return (data[0] & 0xFF) == 0x59 && (data[1] & 0xFF) == 0x59;
  }

  private int parseDistance(byte[] data) {
    // Distance is in bytes 2 and 3
    return (data[2] & 0xFF) | ((data[3] & 0xFF) << 8);
  }

  private int parseStrength(byte[] data) {
    // Strength is in bytes 4 and 5
    return (data[4] & 0xFF) | ((data[5] & 0xFF) << 8);
  }

  private double parseTemperature(byte[] data) {
    // Temperature is in bytes 6 and 7
    int tempRaw = (data[6] & 0xFF) | ((data[7] & 0xFF) << 8);
    // Convert to Celsius
    return (tempRaw / 8.0) - 256.0;
  }

  public int getDistance() {
    return distance;
  }

  public int getStrength() {
    return strength;
  }

  public double getTemperature() {
    return temperature;
  }

  public void periodic() {
    updateSensorData();
  }
}
