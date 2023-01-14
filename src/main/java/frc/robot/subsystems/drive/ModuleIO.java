package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double drivePosition = 0.0;
    public double driveVelocity = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrent = new double[] {};
    public double[] driveTemp = new double[] {};

    public double turnPositionAbsolute = 0.0;
    public double turnPosition = 0.0;
    public double turnVelocity = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrent = new double[] {};
    public double[] turnTemp = new double[] {};
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setDriveVoltage(double volts) {}

  default void setTurnVoltage(double volts) {}

  default void setDriveBrakeMode(boolean brake) {}

  default void setTurnBrakeMode(boolean brake) {}
}
