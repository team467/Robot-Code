package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooterLeaderVelocityRadPerSec;
    public double shooterFollowerVelocityRadPerSec;
    public double indexerVelocityRadPerSec;
    public boolean limitSwitchPressed;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVoltage(double volts) {}

  default void setIndexerVoltage(double volts) {}
}
