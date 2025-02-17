package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double speed = 0.0;
    public double volts = 0.0;
    public double current = 0.0;
    public double position = 0.0;
    public boolean climberDeployed = false;
    public boolean climberWinched = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setSpeed(double speed) {}

  default void setVoltage(double volts) {}

  default void resetPosition() {}

  default void hold() {}
}
