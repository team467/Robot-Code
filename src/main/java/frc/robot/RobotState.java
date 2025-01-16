package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Robot state will hold all the global information that is shared across the subsystems, such as
 * leds or for the shooter to know if the system has a game piece in the robot.
 */
public class RobotState {
  /** Set by the robot if the batter is less than 9V */
  @AutoLogOutput(key = "RobotState/LowBatteryAlert")
  public boolean lowBatteryAlert = false;

  /** The singleton instance of the RobotState class. */
  private static RobotState instance = null;

  @AutoLogOutput(key = "RobotState/ClimberUp")
  public boolean climberUp = false;

  @AutoLogOutput(key = "RobotState/ClimberDown")
  public boolean climberDown = false;

  @AutoLogOutput(key = "RobotState/ClimberRatchet")
  public boolean climberRatchet = true;

  /**
   * Ensures there is only a single instance of the Robot State class.
   *
   * @return the RobotState instance
   */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
}
