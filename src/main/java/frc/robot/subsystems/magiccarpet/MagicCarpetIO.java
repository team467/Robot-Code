package frc.robot.subsystems.magiccarpet;

import org.littletonrobotics.junction.AutoLog;

public interface MagicCarpetIO {

  @AutoLog
  class MagicCarpetIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double motorVelocity = 0.0; // RPM
  }

  default void updateInputs(MagicCarpetIOInputs inputs) {}
  ;

  /** Set motor speed from 0.0 to 1.0 */
  default void setSpeed(double speed) {}
  ;

  /** Stop the motor */
  default void stop() {}
  ;
}
