package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double middleMotorVelocityRadPerSec;
    public double middleMotorAppliedVolts;
    public double middleMotorCurrentAmps;

    public double bottomMotorVelocityRadPerSec;
    public double bottomMotorAppliedVolts;
    public double bottomMotorCurrentAmps;

    public double topMotorVelocityRadPerSec;
    public double topMotorAppliedVolts;
    public double topMotorCurrentAmps;

    public double totalAmps;

    public double shooterWheelVelocityRadPerSec;
    public double shooterWheelPosition;

    public boolean atSetpoint = false;
    public double setpointRadPerSec = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
