package frc.robot.subsystems.HopperBelt;

import org.littletonrobotics.junction.AutoLog;

public interface HopperBeltIO {

  // Update motor/sensor inputs (called every 20 ms)
  void updateInputs(HopperBeltIOInputs inputs);

  // Set motor speed from 0.0 to 1.0
  void setSpeed(double speed);

  // Stop the motor
  void stop();

  // motor data
  @AutoLog
  class HopperBeltIOInputs {
    public double appliedOutput = 0.0;
    public double motorCurrent = 0.0;
    public double motorVelocity = 0.0; // RPM
  }
}
