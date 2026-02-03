package frc.robot.subsystems.hopperbelt;

import org.littletonrobotics.junction.AutoLog;

public interface HopperBeltIO {

  default void updateInputs(HopperBeltIOInputs inputs) {}
  ;

  /** Set motor speed from 0.0 to 1.0 */
  default void setSpeed(double speed) {}
  ;

  /** Stop the motor */
  default void stop() {}
  ;

  @AutoLog
  class HopperBeltIOInputs {
    public double appliedOutput = 0.0;
    public double motorCurrent = 0.0;
    public double motorVelocity = 0.0; // RPM
  }
}
