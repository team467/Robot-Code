package frc.robot.subsystems.robotstate;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Robot state will hold all the global information that is shared across the subsystems, such as
 * leds or for the shooter to know if the system has a game piece in the robot.
 */
public class RobotState {

  /** Set by the robot if the batter is less than 9V */
  @AutoLogOutput(key = "RobotState/LowBatteryAlert")
  public boolean lowBatteryAlert = false;

  /** Set true if the robot is firing a game piece. */
  @AutoLogOutput(key = "RobotState/Shooting")
  public boolean shooting = false;

  /** Set true if the robot is currently intaking a game piece. */
  @AutoLogOutput(key = "RobotState/Intaking")
  public boolean intaking = false;

  /** Set true if the robot is currently hanging from the chain or in the process of climbing. */
  @AutoLogOutput(key = "RobotState/Hanging")
  public boolean hanging = false;

  /**
   * Set true if the robot currently contains a note. Used to prevent the robot from picking up a
   * second note.
   */
  @AutoLogOutput(key = "RobotState/HasNote")
  public boolean hasNote = false;

  /** Set true if the not angle data is valid */
  @AutoLogOutput(key = "RobotState/SeeNote")
  public boolean seeNote = false;

  /** Angle from the front of the robot to a note on the floor in degrees */
  @AutoLogOutput(key = "RobotState/NoteAngle")
  public double noteAngle = 0;

  /** The singleton instance of the RobotState class. */
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
