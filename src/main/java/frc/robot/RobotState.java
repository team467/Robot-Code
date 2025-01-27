package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.leds.LedPatterns;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Robot state will hold all the global information that is shared across the subsystems, such as
 * leds or for the shooter to know if the system has a game piece in the robot.
 */
public class RobotState {

  public enum Mode {

    // 2025 Specific
    ALIGNED_TO_REEF(LedPatterns.RED.colorPatternOnly()),
    ALGAE_EFFECTOR_EXTENDED(LedPatterns.RED.colorPatternOnly()),
    ALGAE_EFFECTOR_RUNNING(LedPatterns.RED.colorPatternOnly()),
    CLIMBER_UP(LedPatterns.RED.colorPatternOnly()),
    DUCK(LedPatterns.RED.colorPatternOnly()),
    COLLISION_DETECTED(LedPatterns.RED.colorPatternOnly()),

    // Same every year
    ESTOPPED(LedPatterns.RED.colorPatternOnly()),
    AUTO_FINISHED(LedPatterns.RED.colorPatternOnly()),
    AUTONOMOUS(LedPatterns.RED.colorPatternOnly()),
    BLUE_ALLIANCE(LedPatterns.RED.colorPatternOnly()),
    RED_ALLIANCE(LedPatterns.RED.colorPatternOnly()),
    LOW_BATTERY_ALERT(LedPatterns.RED.colorPatternOnly()),
    DISABLED(LedPatterns.RED.colorPatternOnly()),
    OFF(LedPatterns.BLACK.colorPatternOnly()),
    DEFAULT(LedPatterns.BLACK.colorPatternOnly());

    public final LEDPattern ledPattern;

    private Mode(LEDPattern ledPattern) {
      this.ledPattern = ledPattern;
    }
  }

  @AutoLogOutput(key = "RobotState/Mode")
  private Mode mode = Mode.OFF;

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
  /** If the algae effector is stowed */
  @AutoLogOutput(key = "RobotState/AlgaeEffectorStowed")
  public boolean algaeEffectorStowed = false;
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

  public void updateState() {
    if (DriverStation.isEStopped()) {
      mode = Mode.ESTOPPED;
    } else if (lowBatteryAlert) {
      mode = Mode.LOW_BATTERY_ALERT;
    } else if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          mode = Mode.BLUE_ALLIANCE;
        } else {
          mode = Mode.RED_ALLIANCE;
        }
      } else {
        mode = Mode.DISABLED;
      }
    } else if (false) {
      mode = Mode.AUTO_FINISHED;
    } else if (DriverStation.isAutonomous()) {
      mode = Mode.AUTONOMOUS;
    } else {
      mode = Mode.DEFAULT;
    }
  }

  public Mode getMode() {
    return mode;
  }
}
