package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Leds extends SubsystemBase {

  private RobotState state = RobotState.getInstance();

  private static SendableChooser<LedPatterns> ledTestPatterns = new SendableChooser<LedPatterns>();

  private LedPatterns pattern;

  // public enum RobotStates {
  //   ALIGNED_TO_REEF,
  //   ALGAE_EFFECTOR_EXTENDED,
  //   ALGAE_EFFECTOR_RUNNING,
  //   CLIMBER_UP,
  //   DUCK,
  //   COLLISION_DETECTED,
  //   ESTOPPED,
  //   AUTO_FINISHED,
  //   OFF,
  //   AUTONOMOUS,
  //   LOW_BATTERY_ALERT,
  //   BLUE_ALLIANCE,
  //   RED_ALLIANCE,
  //   DISABLED,
  //   DEFAULT,

  //   public final LEDPattern pattern;

  //   private LedMode(LEDPattern pattern) {
  //     this.pattern = pattern;
  //   }
  // }

  // @AutoLogOutput(key = "LEDs/Mode")
  // LedMode mode = LedMode.OFF;

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
    ledTestingEntry = ledTable.getEntry("Testing");
    ledTestingEntry.setBoolean(false);

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
              LedPatterns.BREATH.pattern.applyTo(buffer);
              leds.setData(buffer);
            });
    loadingNotifier.startPeriodic(0.02);
  }

  // private void updateState() {
  //   // Update auto state
  //   lastEnabledAuto = DriverStation.isAutonomous();
  //   lastEnabledTime = Timer.getFPGATimestamp();

  //   if (DriverStation.isEStopped()) {
  //     mode = LedMode.ESTOPPED;
  //   } else if (state.lowBatteryAlert) {
  //     mode = LedMode.LOW_BATTERY_ALERT;
  //   } else if (DriverStation.isDisabled()) {
  //     if (DriverStation.getAlliance().isPresent()) {
  //       if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
  //         mode = LedMode.BLUE_ALLIANCE;
  //       } else {
  //         mode = LedMode.RED_ALLIANCE;
  //       }
  //     } else {
  //       mode = LedMode.DISABLED;
  //     }
  //   } else if (false) { // TODO: Test this
  //     mode = LedMode.AUTO_FINISHED;
  //   } else if (DriverStation.isAutonomous()) {
  //     mode = LedMode.AUTONOMOUS;
  //   } else {
  //     mode = LedMode.DEFAULT;
  //   }
  // }

  @Override
  public void periodic() {
    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < LedConstants.MIN_LOOP_CYCLE_COUNT) {
      return;
    }

    loadingNotifier.stop();

    System.out.println(
        "check booleans " + DriverStation.isTest() + " : " + ledTestingEntry.getBoolean(false));
    if (DriverStation.isTest()) {
      try {
        System.out.println("setting pattern to " + ledTestPatterns.getSelected().toString());
        pattern = LedPatterns.valueOf(ledTestPatterns.getSelected().toString());
      } catch (IllegalArgumentException E) {

      }
    } else {
      pattern = LedPatterns.BREATH; // TODO: connect to state
    }

    System.out.println(
        "applying pattern to "
            + pattern.name()
            + " : "
            + pattern.pattern.toString()
            + " : "
            + ledTestPatterns.getSelected().toString());
    pattern.pattern.applyTo(buffer);
    leds.setData(buffer);
  }
}
