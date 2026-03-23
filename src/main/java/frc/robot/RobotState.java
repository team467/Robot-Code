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
    // 2026 Specific
    INTAKING(LedPatterns.YELLOW.blink(0.2).atBrightness(Percent.of(80)), Sections.FULL),
    // INTAKE_MOVING_IN(LedPatterns.YELLOW.blink(0.5).atBrightness(Percent.of(50)), Sections.FULL),
    // INTAKE_MOVING_OUT(LedPatterns.YELLOW.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),
    // INTAKE_DEPLOYED(LedPatterns.CENTER_OF_MASS_GOLD.colorPatternOnly().atBrightness(Percent.of(50)),Sections.FULL),
    // INTAKE_STOWED(LedPatterns.CENTER_OF_MASS_BLUE.colorPatternOnly().atBrightness(Percent.of(50)),Sections.FULL),

    SHOOTER_SPINNING_UP(
        LedPatterns.WHITE.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    SHOOTER_AT_SPEED(LedPatterns.WHITE.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),

    INDEXER_RUNNING(
        LedPatterns.CENTER_OF_MASS_BLUE.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),

    INTAKING_INDEXING_SHOOTING(
        LedPatterns.STRIPE_YELOW_WHITE_BLUE.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),
    INTAKING_INDEXING(
        LedPatterns.STRIPE_YELLOW_BLUE.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),
    INTAKING_SHOOTING(
        LedPatterns.STRIPE_YELLOW_WHITE.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),
    INDEXING_SHOOTING(
        LedPatterns.STRIPE_BLUE_WHITE.blink(0.2).atBrightness(Percent.of(50)), Sections.FULL),

    BROWNING_OUT(
        LedPatterns.INDIAN_RED.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    IN_SHOOTING_RANGE(
        LedPatterns.STRIPE_COM.colorPatternOnly().atBrightness(Percent.of(50)), Sections.FULL),
    OUTOF_SHOOTING_RANGE(LedPatterns.RED.blink().atBrightness(Percent.of(50)), Sections.FULL),

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

  public enum IntakePosition {
    STOWED,
    DEPLOYED,
    MOVING_OUT,
    MOVING_IN
  }

  public enum ClimberPosition {
    STOWED,
    DEPLOYED,
    MOVING_OUT,
    MOVING_IN
  }

  @AutoLogOutput(key = "RobotState/Mode")
  private Mode mode = Mode.OFF;

  /** Set by the robot if the batter is less than 9V */
  @AutoLogOutput(key = "RobotState/LowBatteryAlert")
  public boolean lowBatteryAlert = false;

  @AutoLogOutput(key = "RobotState/PoseConfidence")
  public boolean PoseConfidence = false;

  @AutoLogOutput(key = "RobotState/Intaking")
  public boolean intaking = false;

  @AutoLogOutput(key = "RobotState/IntakePosition")
  public IntakePosition intakePosition = IntakePosition.STOWED;

  @AutoLogOutput(key = "RobotState/ClimberPosition")
  public ClimberPosition climberPosition = ClimberPosition.STOWED;

  @AutoLogOutput(key = "RobotState/IndexerHasFuel")
  public boolean indexerHasFuel = false;

  @AutoLogOutput(key = "RobotState/IndexerRunning")
  public boolean indexerRunning = false;

  @AutoLogOutput(key = "RobotState/ShooterAtSpeed")
  public boolean shooterAtSpeed = false;

  @AutoLogOutput(key = "RobotState/IsAlignedToHub")
  public boolean isAlignedToHub = false;

  @AutoLogOutput(key = "RobotState/ShooterSetpoint")
  public double shooterSetpoint = 0.0;

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

    // check combinations of shooter and indexer and intaking states
    if (intaking && indexerRunning && shooterAtSpeed) {
      mode = Mode.INTAKING_INDEXING_SHOOTING;
      return;
    } else if (intaking && indexerRunning) {
      mode = Mode.INTAKING_INDEXING;
      return;
    } else if (intaking && shooterAtSpeed) {
      mode = Mode.INTAKING_SHOOTING;
      return;
    } else if (indexerRunning && shooterAtSpeed) {
      mode = Mode.INDEXING_SHOOTING;
      return;
    }

    // Intake
    if (intaking) {
      mode = Mode.INTAKING;
      return;
    } // else if (intakePosition == IntakePosition.MOVING_IN) {
    // mode = Mode.INTAKE_MOVING_IN;
    // return;
    // } else if (intakePosition == IntakePosition.MOVING_OUT) {
    //  mode = Mode.INTAKE_MOVING_OUT;
    //  return;
    // } else if (intakePosition == IntakePosition.DEPLOYED) {
    //  mode = Mode.INTAKE_DEPLOYED;
    //  return;
    // } else if (intakePosition == IntakePosition.STOWED) {
    //  mode = Mode.INTAKE_STOWED;
    //  return;
    // }

    if (shooterAtSpeed) {
      mode = Mode.SHOOTER_AT_SPEED;
      return;
    } else if (shooterSetpoint > 0.0) {
      mode = Mode.SHOOTER_SPINNING_UP;
      return;
    }

    // Default
    mode = Mode.DEFAULT;
  }

  public Mode getMode() {
    return mode;
  }
}
