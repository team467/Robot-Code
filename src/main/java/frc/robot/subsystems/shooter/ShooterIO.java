package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooterVelocityRadPerSec;
    public double indexerVelocityRadPerSec;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setFlywheelVoltage(double volts) {}

  default void setIndexerVoltage(double volts) {}
}
