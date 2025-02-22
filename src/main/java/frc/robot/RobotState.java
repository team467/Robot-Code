package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.leds.LedPatterns;
import frc.robot.subsystems.leds.Leds.Sections;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Robot state will hold all the global information that is shared across the subsystems, such as
 * leds or for the shooter to know if the system has a game piece in the robot.
 */
public class RobotState {

  public enum Mode {

    // 2025 Specific
    HAS_CORAL(LedPatterns.PURPLE.colorPatternOnly(), Sections.FULL),
    HOPPER_SEES_CORAL(LedPatterns.PURPLE.colorPatternOnly(), Sections.BASE1),
    INTAKING_CORAL(LedPatterns.YELLOW.blink(0.2), Sections.FULL),
    ALGAE_EFFECTOR_RUNNING(LedPatterns.GREEN.scroll(), Sections.FULL),
    CLIMBER_WINCHED(LedPatterns.RAINBOW.scroll(), Sections.FULL),
    DUCK(LedPatterns.STRIPE_COM.scroll(), Sections.FULL),
    ELEVATOR_CORAL_L1(LedPatterns.PURPLE.colorPatternOnly(), Sections.FIRST_QUARTER),
    ELEVATOR_CORAL_L2(LedPatterns.PURPLE.colorPatternOnly(), Sections.SECOND_QUARTER),
    ELEVATOR_CORAL_L3(LedPatterns.PURPLE.colorPatternOnly(), Sections.THIRD_QUARTER),
    ELEVATOR_CORAL_L4(LedPatterns.PURPLE.colorPatternOnly(), Sections.FOURTH_QUARTER),
    ELEVATOR_ALGAE_L2(LedPatterns.GREEN.colorPatternOnly(), Sections.SECOND_QUARTER),
    ELEVATOR_ALGAE_L3(LedPatterns.GREEN.colorPatternOnly(), Sections.THIRD_QUARTER),

    // Same every year
    ESTOPPED(LedPatterns.RED.colorPatternOnly(), Sections.FULL),
    AUTO_FINISHED(LedPatterns.STRIPE_COM.colorPatternOnly(), Sections.FULL),
    AUTONOMOUS(LedPatterns.RAINBOW.colorPatternOnly(), Sections.FULL),
    BLUE_ALLIANCE(LedPatterns.BLUE.colorPatternOnly(), Sections.FULL),
    RED_ALLIANCE(LedPatterns.RED.colorPatternOnly(), Sections.FULL),
    LOW_BATTERY_ALERT(LedPatterns.RED.blink(0.2), Sections.FULL),
    DISABLED(LedPatterns.GRAY.colorPatternOnly(), Sections.FULL),
    OFF(LedPatterns.BLACK.colorPatternOnly(), Sections.FULL),
    DEFAULT(LedPatterns.BLACK.colorPatternOnly(), Sections.FULL);

    public final LEDPattern ledPattern;
    public final Sections ledSection;

    private Mode(LEDPattern ledPattern, Sections section) {
      this.ledPattern = ledPattern;
      this.ledSection = section;
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

  @AutoLogOutput(key = "RobotState/IntakingCoral")
  public boolean intakingCoral = false;
  /** Robot dumping coral */
  @AutoLogOutput(key = "RobotState/SendCoral")
  public boolean dumpingCoral = false;
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
  /** If the robot is tilting */
  @AutoLogOutput(key = "RobotState/RobotTilted")
  public boolean robotTilted = false;
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
