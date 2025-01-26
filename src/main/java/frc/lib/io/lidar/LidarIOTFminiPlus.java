package frc.lib.io.lidar;

import edu.wpi.first.wpilibj.SerialPort;
import frc.lib.io.lidar.driver.TFminiPlus;

public class LidarIOTFminiPlus implements LidarIO {
  private final TFminiPlus tfminiPlus;

  public LidarIOTFminiPlus(SerialPort.Port port) {
    this.tfminiPlus = new TFminiPlus(port);
  }

  @Override
  public void updateInputs(LidarIOInputs inputs) {
    tfminiPlus.periodic();
    inputs.distanceMeters = tfminiPlus.getDistance();
    inputs.strength = tfminiPlus.getStrength();
    inputs.temperatureCelcius = tfminiPlus.getTemperature();
  }
}
