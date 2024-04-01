package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Robot state will hold all the global information that is shared across the subsystems, such as
 * leds or for the shooter to know if the system has a game piece in the robot.
 */
public class RobotState {

  /** Set true if robot is in range of the speaker */
  @AutoLogOutput(key = "RobotState/InRange")
  public boolean inRange = false;

  /** Set true if robot can shoot into speaker */
  @AutoLogOutput(key = "RobotState/CanShoot")
  public boolean canShoot = false;

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
  @AutoLogOutput(key = "RobotState/ClimberUp")
  public boolean climberUp = false;

  @AutoLogOutput(key = "RobotState/ClimberDown")
  public boolean climberDown = false;

  @AutoLogOutput(key = "RobotState/ClimberRatchet")
  public boolean climberRatchet = true;

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

  /** If the duck button is pressed */
  @AutoLogOutput(key = "RobotState/Duck")
  public boolean duck = false;

  /** If the shooter is at the speed needed to shoot into the speaker */
  @AutoLogOutput(key = "RobotState/ShooterSpeedReady")
  public boolean shooterSpeedIsReady = false;

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
