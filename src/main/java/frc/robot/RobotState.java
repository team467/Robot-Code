package frc.robot;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.geometry.Pose2d;
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
    HAS_CORAL(LedPatterns.PURPLE.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    HOPPER_SEES_CORAL(
        LedPatterns.PURPLE.colorPatternOnly().atBrightness(Percent.of(50)), Sections.BASE1),
    INTAKING_CORAL(LedPatterns.STRIPE_CORAL.scroll(2).atBrightness(Percent.of(50)), Sections.FULL),
    ALGAE_EFFECTOR_RUNNING(
        LedPatterns.STRIPE_ALGAE.scroll(2).atBrightness(Percent.of(50)), Sections.FULL),
    CLIMBER_WINCHED(LedPatterns.RAINBOW.scroll().atBrightness(Percent.of(50)), Sections.FULL),
    DUCK(LedPatterns.STRIPE_COM.scroll().atBrightness(Percent.of(50)), Sections.FULL),
    ELEVATOR_CORAL_L1(
        LedPatterns.PURPLE.blink(0.2).atBrightness(Percent.of(50)), Sections.FIRST_QUARTER),
    ELEVATOR_CORAL_L2(
        LedPatterns.PURPLE.blink(0.2).atBrightness(Percent.of(50)), Sections.SECOND_QUARTER),
    ELEVATOR_CORAL_L3(
        LedPatterns.PURPLE.blink(0.2).atBrightness(Percent.of(50)), Sections.THIRD_QUARTER),
    ELEVATOR_CORAL_L4(
        LedPatterns.PURPLE.blink(0.2).atBrightness(Percent.of(50)), Sections.FOURTH_QUARTER),
    ELEVATOR_ALGAE_L2(
        LedPatterns.GREEN.blink(0.2).atBrightness(Percent.of(50)), Sections.SECOND_QUARTER),
    ELEVATOR_ALGAE_L3(
        LedPatterns.GREEN.blink(0.2).atBrightness(Percent.of(50)), Sections.THIRD_QUARTER),

    // Same every year
    ESTOPPED(LedPatterns.RED.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    AUTO_FINISHED(
        LedPatterns.STRIPE_COM.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    AUTONOMOUS(LedPatterns.RAINBOW.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    BLUE_ALLIANCE(
        LedPatterns.FRC_BLUE.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    RED_ALLIANCE(
        LedPatterns.FRC_RED.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    LOW_BATTERY_ALERT(LedPatterns.RED.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),
    DISABLED(LedPatterns.GRAY.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    OFF(LedPatterns.BLACK.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    DEFAULT(LedPatterns.BLACK.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL);

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
  /** If the fast algae effector is stowed */
  @AutoLogOutput(key = "RobotState/FastAlgaeEffectorStowed")
  public boolean fastAlgaeEffectorStowed = false;
  /** If the fast algae effector is high */
  @AutoLogOutput(key = "RobotState/FastAlgaeEffectorHigh")
  public boolean fastAlgaeEffectorHigh = false;
  /** If the fast algae effector is low */
  @AutoLogOutput(key = "RobotState/FastAlgaeEffectorLow")
  public boolean fastAlgaeEffectorLow = false;
  /** If the climber is up */
  @AutoLogOutput(key = "RobotState/ClimberWinched")
  public boolean climberWinched = false;
  /** If the climber is down */
  @AutoLogOutput(key = "RobotState/ClimberDeployed")
  /** If the climber is deployed */
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

  @AutoLogOutput(key = "RobotState/PoseConfidence")
  public boolean PoseConfidence = false;

  @AutoLogOutput(key = "RobotState/FastAlgaeStowed")
  public boolean fastAlgaeStowed = false;

  @AutoLogOutput(key = "RobotState/targetPose")
  public Pose2d targetPose = new Pose2d();

  @AutoLogOutput(key = "RobotState/ClosestReefFace")
  public int ClosestReefFace = 0;

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
    L1,
    L2,
    L3,
    L4,
    ALGAE_L2,
    ALGAE_L3
  }

  public void updateLEDState() {
    // Emergency
    if (DriverStation.isEStopped()) {
      mode = Mode.ESTOPPED;
      return;
    }
    if (lowBatteryAlert) {
      mode = Mode.LOW_BATTERY_ALERT;
      return;
    }

    // Disabled
    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          mode = Mode.BLUE_ALLIANCE;
        } else {
          mode = Mode.RED_ALLIANCE;
        }
      } else {
        mode = Mode.DISABLED;
      }
      return;
    }

    // Auto
    if (DriverStation.isAutonomous()) {
      mode = Mode.AUTONOMOUS;
      return;
    }

    // Climber
    if (climberWinched) {
      mode = Mode.CLIMBER_WINCHED;
      return;
    }

    // Duck
    if (duck) {
      mode = Mode.DUCK;
      return;
    }

    // Elevator
    if (elevatorPosition != null) {
      switch (elevatorPosition) {
        case L1:
          mode = Mode.ELEVATOR_CORAL_L1;
          return;
        case L2:
          mode = Mode.ELEVATOR_CORAL_L2;
          return;
        case L3:
          mode = Mode.ELEVATOR_CORAL_L3;
          return;
        case L4:
          mode = Mode.ELEVATOR_CORAL_L4;
          return;
        case ALGAE_L2:
          mode = Mode.ELEVATOR_ALGAE_L2;
          return;
        case ALGAE_L3:
          mode = Mode.ELEVATOR_ALGAE_L3;
          return;
        default:
          break;
      }
    }

    // Coral actions
    if (hopperSeesCoral) {
      mode = Mode.HOPPER_SEES_CORAL;
      return;
    }
    if (intakingCoral) {
      mode = Mode.INTAKING_CORAL;
      return;
    }

    // Algae
    if (algaeEffectorRunning) {
      mode = Mode.ALGAE_EFFECTOR_RUNNING;
      return;
    }

    // Has coral (lowest)
    if (hasCoral) {
      mode = Mode.HAS_CORAL;
      return;
    }

    // Default
    mode = Mode.DEFAULT;
  }

  public Mode getMode() {
    return mode;
  }
}
