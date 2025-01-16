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
    BASE,
    BASE2
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

      case BASE:
        LedPatterns.BASE.applyTo(buffer);
        break;

      case OFF:
        LedPatterns.OFF.applyTo(buffer);
        break;

      case BASE2:
        LedPatterns.BASE2.applyTo(buffer);
        break;

      default:
    }

    leds.setData(buffer);
  }

  @Override
  public void periodic() {
    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < LedConstants.MIN_LOOP_CYCLE_COUNT) {
      return;
    }

    loadingNotifier.stop();

    if (ledTestingEntry.getBoolean(false)) {
      try {
        mode = LedMode.valueOf(ledModeEntry.getString("OFF"));
      } catch (IllegalArgumentException E) {

      }
    } else {
      updateState();
      ledModeEntry.setString(mode.toString());
    }
    updateLeds();
  }
}
