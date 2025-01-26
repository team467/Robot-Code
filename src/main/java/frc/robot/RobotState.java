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
  /** If the coral effector limit switch is pressed */
  @AutoLogOutput(key = "RobotState/HasCoral")
  public boolean hasCoral = false;
  /** If the hopper optical sensor is triggered */
  @AutoLogOutput(key = "RobotState/HopperSeesCoral")
  public boolean hopperSeesCoral = false;
  /** If the robot is aligned to the reef */
  @AutoLogOutput(key = "RobotState/AlignedToReef")
  public boolean alignedToReef = false;
  /** If the algae effector is extended */
  @AutoLogOutput(key = "RobotState/AlgaeEffectorExtended")
  public boolean algaeEffectorExtended = false;
  /** If the algae effector is running */
  @AutoLogOutput(key = "RobotState/AlgaeEffectorRunning")
  public boolean algaeEffectorRunning = false;
  /** If the climber is up */
  @AutoLogOutput(key = "RobotState/ClimberWinched")
  public boolean climberWinched = false;
  /** If the climber is down */
  @AutoLogOutput(key = "RobotState/ClimberDeployed")
  public boolean climberDeployed = false;

  @AutoLogOutput(key = "RobotState/ClimberStowed")
  public boolean climberStowed = false;
  /** If the robot is in currently ducking */
  @AutoLogOutput(key = "RobotState/Duck")
  public boolean duck = false;
  /** If the robot has detected a collision */
  @AutoLogOutput(key = "RobotState/CollisionDetected")
  public boolean collisionDetected = false;
  /** the position that the elevator is at */
  @AutoLogOutput(key = "RobotState/ElevatorPosition")
  public ElevatorPosition elevatorPosition = null;

  @AutoLogOutput(key = "RobotState/AlgaeAffectorExtended")
  public boolean algaeAffectorExtended = false;

  @AutoLogOutput(key = "RobotState/AlgaeMotorSpinning")
  public boolean algaeMotorSpinning = false;

  @AutoLogOutput(key = "RobotState/AlgaeAffectorStowed")
  public boolean algaeEffectorStowed = false;

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

  public enum ElevatorPosition {
    INTAKE,
    CORAL_LEVEL_1,
    CORAL_LEVEL_2,
    CORAL_LEVEL_3,
    CORAL_LEVEL_4,
    ALGAE_LEVEL_2,
    ALGAE_LEVEL_3;
  }
}
