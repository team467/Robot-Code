package frc.robot.subsystems.robotstate;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.AutoLog;

public interface RobotStateIO {

  @AutoLog
  public class RobotStateIOInputs {
    public DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    public boolean estopped = false;
    public boolean lowBatteryAlert = false;
  }

  default void initialize(RobotStateIOInputs inputs) {}

  default void updateInputs(RobotStateIOInputs inputs) {}
}
