package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double middleMotorRPM;
    public double middleMotorAppliedVolts;
    public double middleMotorCurrentAmps;

    public double shooterLeaderVelocityRPM;
    public double shooterLeaderAppliedVolts;
    public double shooterLeaderCurrentAmps;

    public double shooterFollowerVelocityRPM;
    public double shooterFollowerAppliedVolts;
    public double shooterFollowerCurrentAmps;

    public boolean atSetpoint = false;
    public double setpointRPM = 0;

    public double shooterFollower2VelocityRPM;
    public double shooterFollower2AppliedVolts;
    public double shooterFollower2CurrentAmps;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void setVoltage(double volts) {}

  default void stop() {}

  default void setTargetVelocity(double setpoint) {}

  default void goToSetpoint() {}

  default void setTargetDistance(double distanceMeters) {}
}
