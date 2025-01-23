package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralEffectorIO {

  @AutoLog
  class EffectorIOInputs {

    public double appliedVolts;
    public double currentAmps;
    public double temperature;
    public boolean coralOnTheWay = false;
    public boolean haveCoral = false;
    public boolean motorLimitSwitch = false;
  }

  default void updateInputs(EffectorIOInputs inputs) {}

  default void setSpeed(double speed) {}

  default void setCoralGone(boolean coralGoneState) {}
}
