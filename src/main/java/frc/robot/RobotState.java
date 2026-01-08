package frc.robot;

import static edu.wpi.first.units.Units.Percent;

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
    //2026 Specific

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

  @AutoLogOutput(key = "RobotState/PoseConfidence")
  public boolean PoseConfidence = false;

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
    HOME,
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

    // Default
    mode = Mode.DEFAULT;
  }

  public Mode getMode() {
    return mode;
  }
}
