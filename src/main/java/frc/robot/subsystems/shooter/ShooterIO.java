package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooterLeaderVelocityRadPerSec;
    public double shooterLeaderAppliedVolts;
    public double shooterLeaderCurrentAmps;
    public double shooterFollowerVelocityRadPerSec;
    public double shooterFollowerAppliedVolts;
    public double shooterFollowerCurrentAmps;
    public double distanceFromHub;
    public double ShootingAngle;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVoltage(double volts) {}

  default void setShooterVelocity(double velocity) {}
}
