package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double middleMotorRPM;
    public double middleMotorAppliedVolts;
    public double middleMotorCurrentAmps;

    public double bottomMotorRPM;
    public double bottomMotorCurrentAmps;
    public double bottomMotorAppliedVolts;

    public boolean atSetpoint = false;
    public double setpointRPM = 0;

    public double topMotorRPM;
    public double topMotorAppliedVolts;
    public double topMotorCurrentAmps;

    public double totalAmps;

    public double shooterRPM;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void setVoltage(double volts) {}

  default void stop() {}

  default void setTargetVelocity(double setpoint) {}

  default void goToSetpoint() {}

  default void setTargetDistance(double distanceMeters) {}

  default boolean isAtSetpoint() {
    return false;
  }
}
