package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double topLeftMotorVelocityRadPerSec;
    public double topLeftMotorAppliedVolts;
    public double topLeftMotorCurrentAmps;

    public double topRightMotorVelocityRadPerSec;
    public double topRightMotorAppliedVolts;
    public double topRightMotorCurrentAmps;

    public double bottomLeftMotorVelocityRadPerSec;
    public double bottomLeftMotorAppliedVolts;
    public double bottomLeftMotorCurrentAmps;

    public double bottomRightMotorVelocityRadPerSec;
    public double bottomRightMotorAppliedVolts;
    public double bottomRightMotorCurrentAmps;

    public boolean atSetpoint = false;
    public double setpointRPM = 0;

    public double totalAmps;

    public double shooterWheelVelocityRadPerSec;
    public double shooterWheelPosition;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
