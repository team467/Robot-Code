package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralEffectorIO {

  @AutoLog
  class CoralEffectorIOInputs {

    public double velocity;
    public double appliedVolts;
    public double currentAmps;
    public double temperature;
    public boolean coralOnTheWay = false;
    public boolean haveCoral = false;
  }

  default void updateInputs(CoralEffectorIOInputs inputs) {}

  default void setSpeed(double speed) {}

  
  
}
