package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralEffectorIO {

  @AutoLog
  class CoralEffectorIOInputs {

    public double velocity;
    public double appliedVolts;
    public double currentAmps;
    public double temperature;
    public boolean hopperSeesCoral = false;
    public boolean hasCoral = false;
  }

  default void updateInputs(CoralEffectorIOInputs inputs) {}

  default void setSpeed(double speed) {}
}
