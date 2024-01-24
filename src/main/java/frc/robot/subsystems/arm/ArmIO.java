package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public Rotation2d positionRads = new Rotation2d();
    public double velocityRadsPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void setVoltage(double volts) {}
}
