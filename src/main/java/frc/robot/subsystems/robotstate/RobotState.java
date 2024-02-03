package frc.robot.subsystems.robotstate;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Robot state will hold all the global information that is shared across the subsystems, such as
 * leds or for the shooter to know if the system has a game piece in the robot.
 */
public class RobotState {

  @AutoLogOutput(key = "RobotState/LowBatteryAlert")
  public boolean lowBatteryAlert = false;

  @AutoLogOutput(key = "RobotState/Shooting")
  public boolean shooting = false;

  @AutoLogOutput(key = "RobotState/Intaking")
  public boolean intaking = false;

  @AutoLogOutput(key = "RobotState/Hanging")
  public boolean hanging = false;

  @AutoLogOutput(key = "RobotState/HasNote")
  public boolean hasNote = false;

  @AutoLogOutput(key = "RobotState/SeeNote")
  public boolean seeNote = false;

  @AutoLogOutput(key = "RobotState/NoteAngle")
  public double noteAngle = 0;

  private static RobotState instance = null;

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
