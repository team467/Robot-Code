package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public double turnPositionAbsoluteRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDriveVoltage(double volts) {}

  default void setTurnVoltage(double volts) {}

  default void setDriveBrakeMode(boolean brake) {}

  default void setTurnBrakeMode(boolean brake) {}
}
