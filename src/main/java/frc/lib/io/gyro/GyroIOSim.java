package frc.lib.io.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

/** A Gyro IO class that pretends to have a set angle. */
public class GyroIOSim implements GyroIO {

  private Supplier<Rotation2d> angle;

  /**
   * Creates a modifiable gyro.
   *
   * @param angle the angle to have the gyro claim
   */
  public GyroIOSim(Supplier<Rotation2d> angle) {
    this.angle = angle;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.angle = angle.get().getDegrees();
    inputs.rate = 0;
  }
}
