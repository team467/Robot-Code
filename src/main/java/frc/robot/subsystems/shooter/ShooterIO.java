package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooterTopVelocityRadPerSec;
    public double shooterTopAppliedVolts;
    public double shooterTopCurrentAmps;
    public double shooterBottomVelocityRadPerSec;
    public double shooterBottomAppliedVolts;
    public double shooterBottomCurrentAmps;
    public double distanceFromSpeaker;
    public double ShootingAngle;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setShooterVoltage(double volts) {}
}
