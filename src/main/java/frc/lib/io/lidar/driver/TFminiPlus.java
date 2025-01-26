package frc.lib.io.lidar.driver;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * TFmini Plus LiDAR Module
 * @see <a href="https://cdn.sparkfun.com/assets/1/4/2/1/9/TFmini_Plus_A02_Product_Manual_EN.pdf">Product Manual</a>
 */
public class TFminiPlus {
  private final SerialPort serialPort;

  private int distance = 0;
  private int strength = 0;
  private double temperature = 0.0;

  /**
   * Creates a new TFmini Plus LiDAR Module
   * @param port RS-232 port
   */
  public TFminiPlus(SerialPort.Port port) {
    serialPort = new SerialPort(115200, port);
  }

  // Reads and parses data from the sensor
  public void periodic() {
    byte[] data = serialPort.read(9);

    if (data.length == 9 && isVaildFrame(data)) {
      distance = parseDistance(data);
      strength = parseStrength(data);
      temperature = parseTemperature(data);
    }
  }

  /**
   * Checks if the provided data frame is valid by verifying the start bytes.
   *
   * @param data the byte array representing the received data frame
   * @return true if the start bytes are 0x59 and 0x59, false otherwise
   */
  private boolean isVaildFrame(byte[] data) {
    // Start bytes must be 0x59 and 0x59
    return (data[0] & 0xFF) == 0x59 && (data[1] & 0xFF) == 0x59;
  }

  /**
   * Parses the distance value from the provided data frame.
   *
   * @param data the byte array representing the received data frame, where bytes 2 and 3 contain the distance value
   * @return the parsed distance value as an integer, in meters
   */
  private int parseDistance(byte[] data) {
    // Distance is in bytes 2 and 3, data is in cm
    return (data[2] & 0xFF) | ((data[3] & 0xFF) << 8) / 100;
  }

  /**
   * Parses the strength value from the provided data frame.
   *
   * @param data the byte array representing the received data frame, where bytes 4 and 5 contain the strength value
   * @return the parsed strength value as an integer
   */
  private int parseStrength(byte[] data) {
    // Strength is in bytes 4 and 5
    return (data[4] & 0xFF) | ((data[5] & 0xFF) << 8);
  }

  /**
   * Parses the temperature value from the provided data frame.
   *
   * @param data the byte array representing the received data frame, where bytes 6 and 7 contain the temperature value
   * @return the parsed temperature value as a double, in degrees Celsius
   */
  private double parseTemperature(byte[] data) {
    // Temperature is in bytes 6 and 7
    int tempRaw = (data[6] & 0xFF) | ((data[7] & 0xFF) << 8);
    // Convert to Celsius
    return (tempRaw / 8.0) - 256.0;
  }

  /**
   * Retrieves the most recently measured distance value by the LiDAR module.
   *
   * @return the last recorded distance value as an integer, in meters
   */
  public int getDistance() {
    return distance;
  }

  /**
   * Retrieves the most recently measured strength value from the LiDAR module.
   *
   * @return the last recorded strength value as an integer
   */
  public int getStrength() {
    return strength;
  }

  /**
   * Retrieves the most recently measured temperature value from the LiDAR module.
   *
   * @return the last recorded temperature value as a double, in degrees Celsius
   */
  public double getTemperature() {
    return temperature;
  }
}
