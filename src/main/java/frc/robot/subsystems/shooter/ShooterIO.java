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

    public double shooterFollower2VelocityRadPerSec;
    public double shooterFollower2AppliedVolts;
    public double shooterFollower2CurrentAmps;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void setVoltage(double volts) {}

  default void stop() {}

  default void setTargetVelocity(double setpoint) {}

  default void goToSetpoint() {}
}
