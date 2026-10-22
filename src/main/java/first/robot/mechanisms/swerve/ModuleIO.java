package first.robot.mechanisms.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnPosition = Rotation2d.ZERO;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double turnPositionAbsolute = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveOpenLoop(double output) {}

  public default void setTurnOpenLoop(double output) {}

  public default void setDriveVelocity(double velocityRadPerSec) {}

  public default void setTurnPosition(Rotation2d rotation) {}
}
