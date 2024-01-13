package frc.robot.subsystems.shooter;


import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    private double shooterVelocityRadPerSec;
    private double indexerVelocityRadPerSec;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVoltage(double volts) {}

  default void setIndexerVoltage(double volts) {
  }

}
