package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.AutoLogOutput;

public class Leds extends SubsystemBase {

  private RobotState state = RobotState.getInstance();

  public enum LedMode {
    ESTOPPED,
    AUTO_FINISHED,
    AUTONOMOUS,
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    LOW_BATTERY_ALERT,
    DISABLED,
    OFF,
    DEFAULT
  }

  @AutoLogOutput(key = "LEDs/Mode")
  LedMode mode = LedMode.OFF;

  // Robot state tracking
  public int loopCycleCount = 0;
  public double autoFinishedTime = 0.0;

  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView left;
  private final AddressableLEDBufferView right;
  private final Notifier loadingNotifier;

  // Constants
  private static final int minLoopCycleCount = 10;
  private static final int length = 40;
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.2;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 3.0;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  /** Creates a Network table for testing led modes and colors */
  private NetworkTable ledTable;
  /** Sets the mode for led in network table and allows to test led modes */
  private NetworkTableEntry ledModeEntry;
  /** Allows testing in leds by enabling testing mode */
  private NetworkTableEntry ledTestingEntry;

  public Leds() {
    ledTable = NetworkTableInstance.getDefault().getTable("Leds");
    ledModeEntry = ledTable.getEntry("Mode");
    ledModeEntry.setString("OFF");
    ledTestingEntry = ledTable.getEntry("Testing");
    ledTestingEntry.setBoolean(false);

    buffer = new AddressableLEDBuffer(length);
    left = buffer.createView(0, length / 2 - 1);
    right = buffer.createView(length / 2, length - 1);

    // define patterns

    leds = new AddressableLED(0);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();

    loadingNotifier =
        new Notifier(
            () -> {
              LedPatterns.BREATH.applyTo(buffer);
              leds.setData(buffer);
            });
    loadingNotifier.startPeriodic(0.02);
  }

  private void updateState() {

    // Update auto state
    lastEnabledAuto = DriverStation.isAutonomous();
    lastEnabledTime = Timer.getFPGATimestamp();

    if (DriverStation.isEStopped()) {
      mode = LedMode.ESTOPPED;
    } else if (state.lowBatteryAlert) {
      mode = LedMode.LOW_BATTERY_ALERT;
      // low battery mode at top for testing purposes
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
    } else if (false) { // TODO: Test this
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
        LedPatterns.ESTOPPED.applyTo(buffer);
        break;

      case AUTO_FINISHED:
        LedPatterns.AUTO_FINISHED.applyTo(buffer);
        break;

      case AUTONOMOUS:
        LedPatterns.AUTONOMOUS.applyTo(buffer);
        break;

      case BLUE_ALLIANCE:
        LedPatterns.BLUE_ALLIANCE.applyTo(buffer);
        break;

      case RED_ALLIANCE:
        LedPatterns.RED_ALLIANCE.applyTo(buffer);
        break;

      case LOW_BATTERY_ALERT:
        LedPatterns.LOW_BATTERY_ALERT.applyTo(buffer);
        break;

      case DISABLED:
        LedPatterns.DISABLED.applyTo(buffer);
        break;

      case DEFAULT:
        LedPatterns.DEFAULT.applyTo(buffer);
        break;

      case OFF:
        LedPatterns.OFF.applyTo(buffer);
        break;

      default:
    }

    leds.setData(buffer);
  }

  @Override
  public void periodic() {

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }
    // Stop loading notifier if running
    loadingNotifier.stop();

    if (ledTestingEntry.getBoolean(false)) {
      mode = LedMode.valueOf(ledModeEntry.getString("OFF"));
    } else {
      updateState();
      ledModeEntry.setString(mode.toString());
    }
    updateLeds();
  }
}
