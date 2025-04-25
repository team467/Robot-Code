package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double turnPositionAbsolute = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}
  // specified by ModuleIO interface
  public default void setDriveOutputVoltage(Voltage voltage) {}

  // specified by ModuleIO interface
  public default void setSteerOutputVoltage(Voltage voltage) {}

  // specified by ModuleIO interface
  public default Rotation2d getSteerFacing() {
    return null;
  }

  // specified by ModuleIO interface
  public default Angle getSteerRelativePosition() {
    return null;
  }
  // specified by ModuleIO interface
  public default Angle getDriveWheelPositioned() {
    return null;
  }
}
