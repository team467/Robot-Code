package frc.robot.subsystems.robotstate;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState extends SubsystemBase {

  @AutoLogOutput(key = "RobotState/LowBatteryAlert")
  public boolean lowBatteryAlert = false;

  private static RobotState instance = null;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public void periodic() {
  }
  
}
