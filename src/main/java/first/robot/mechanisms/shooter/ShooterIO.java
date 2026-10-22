package first.robot.mechanisms.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;

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

    public double totalAmps;

    public double busVoltage = 12.0;

    public double shooterWheelVelocityRadPerSec;
    public double shooterWheelPosition;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public static enum ShooterIOOutputMode {
    VOLTAGE,
    STOP
  }

  public static class ShooterIOOutputs {
    @AutoLogOutput(key = "Shooter/Output Mode")
    public ShooterIOOutputMode mode = ShooterIOOutputMode.STOP;

    @AutoLogOutput(key = "Shooter/Target Volts")
    public double targetVolts = 0.0;
  }

  public default void applyOutputs(ShooterIOOutputs outputs) {}
}
