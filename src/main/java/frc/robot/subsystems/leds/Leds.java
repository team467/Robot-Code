package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Leds extends SubsystemBase {

  private RobotState state = RobotState.getInstance();

  public enum LedMode {
    ALIGNED_TO_REEF,
    ALGAE_EFFECTOR_EXTENDED,
    ALGAE_EFFECTOR_RUNNING,
    CLIMBER_UP,
    DUCK,
    COLLISION_DETECTED,
    ESTOPPED,
    AUTO_FINISHED,
    AUTONOMOUS,
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    LOW_BATTERY_ALERT,
    DISABLED,
    OFF,
    DEFAULT,
  }

  // @AutoLogOutput(key = "LEDs/Mode")
  LedMode mode = LedMode.OFF;

  public int loopCycleCount = 0;
  public double autoFinishedTime = 0.0;

  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView left;
  private final AddressableLEDBufferView right;
  private final Notifier loadingNotifier;

  private NetworkTable ledTable;
  private NetworkTableEntry ledModeEntry;
  private NetworkTableEntry ledTestingEntry;
  private SendableChooser<LedPatterns> ledTestPatterns;

  public Leds() {
    ledTable = NetworkTableInstance.getDefault().getTable("Leds");
    ledTestingEntry = ledTable.getEntry("Testing");
    ledTestingEntry.setBoolean(false);

    ledTestPatterns = new SendableChooser<LedPatterns>();
    for (LedPatterns pattern : LedPatterns.values()) {
      ledTestPatterns.addOption(pattern.toString(), pattern);
    }
    Shuffleboard.getTab("LEDs").add(ledTestPatterns);

    buffer = new AddressableLEDBuffer(LedConstants.LENGTH);
    left = buffer.createView(0, LedConstants.LENGTH / 2 - 1);
    right = buffer.createView(LedConstants.LENGTH / 2, LedConstants.LENGTH - 1);

    leds = new AddressableLED(LedConstants.LED_CHANNEL);
    leds.setLength(LedConstants.LENGTH);
    leds.setData(buffer);
    leds.start();

    loadingNotifier =
        new Notifier(
            () -> {
              LedPatterns.RED.colorPatternOnly().applyTo(buffer);
              leds.setData(buffer);
            });
    loadingNotifier.startPeriodic(0.02);
  }

  private void updateState() {
    lastEnabledAuto = DriverStation.isAutonomous();
    lastEnabledTime = Timer.getFPGATimestamp();

    if (DriverStation.isEStopped()) {
      mode = LedMode.ESTOPPED;
    } else if (state.lowBatteryAlert) {
      mode = LedMode.LOW_BATTERY_ALERT;
    } else if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          mode = LedMode.BLUE_ALLIANCE;
        } else {
          mode = LedMode.RED_ALLIANCE;
        }
      } else {
        mode = LedMode.DISABLED;
      }
    } else if (false) {
      mode = LedMode.AUTO_FINISHED;
    } else if (DriverStation.isAutonomous()) {
      mode = LedMode.AUTONOMOUS;
    } else {
      mode = LedMode.DEFAULT;
    }
  }

  private void updateLeds() {
    switch (mode) {
      case ESTOPPED:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case AUTO_FINISHED:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case AUTONOMOUS:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case BLUE_ALLIANCE:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case RED_ALLIANCE:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case LOW_BATTERY_ALERT:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case DISABLED:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case DEFAULT:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case ALIGNED_TO_REEF:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case ALGAE_EFFECTOR_EXTENDED:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case ALGAE_EFFECTOR_RUNNING:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case CLIMBER_UP:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case DUCK:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case COLLISION_DETECTED:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      case OFF:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
        break;
      default:
        LedPatterns.RED.colorPatternOnly().applyTo(buffer);
    }

    leds.setData(buffer);
  }

  @Override
  public void periodic() {
    loopCycleCount += 1;
    if (loopCycleCount < LedConstants.MIN_LOOP_CYCLE_COUNT) {
      return;
    }

    loadingNotifier.stop();
    LEDPattern pattern = LedPatterns.BLACK.colorPatternOnly();

    System.out.println(
        "check booleans " + DriverStation.isTest() + " : " + ledTestingEntry.getBoolean(false));

    if (DriverStation.isTest()) {
      try {
        System.out.println("setting pattern to " + ledTestPatterns.getSelected().toString());
        pattern = ledTestPatterns.getSelected().colorPatternOnly();
      } catch (IllegalArgumentException E) {
      }
    } else {
      pattern = LedPatterns.BLACK.colorPatternOnly(); // TODO: connect to state
    }

    System.out.println(
        "applying pattern to "
            + pattern.toString()
            + " : "
            + pattern.toString()
            + " : "
            + ledTestPatterns.getSelected().toString());

    pattern.applyTo(buffer);
    leds.setData(buffer);
  }
}
