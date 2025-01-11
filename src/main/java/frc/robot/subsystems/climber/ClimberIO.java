package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double motorPercentOutput = 0.0;
    public double motorVoltage = 0.0;
    public double currentAmpsLeader = 0.0;
    public double appliedVoltsLeader = 0.0;
    public double ClimberLeaderPosition = 0.0;
    public double currentAmpsFollower = 0.0;
    public double appliedVoltsFollower = 0.0;
    public double ClimberFollowerPosition = 0.0;
    public boolean ratchetLocked = false;
    public boolean limitSwitchPressed = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setMotorsOutputPercent(double percentOutput) {}

  default void setRatchetLocked(boolean locked) {}

  default void setLeaderMotorPercentOutput(double volts) {}

  default void setFollowerMotorPercentOutput(double volts) {}

  default boolean getLimitSwitch() {
    return true;
  }

  default void resetPosition() {}
}
