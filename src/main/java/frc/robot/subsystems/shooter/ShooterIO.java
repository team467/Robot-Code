package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooterLeftVelocityRadPerSec;
    public double shooterLeftAppliedVolts;
    public double shooterLeftCurrentAmps;
    public double shooterRightVelocityRadPerSec;
    public double shooterRightAppliedVolts;
    public double shooterRightCurrentAmps;
    public double distanceFromSpeaker;
    public double ShootingAngle;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVoltage(double volts) {}

  default void setShooterVelocity(double velocity) {}
}
