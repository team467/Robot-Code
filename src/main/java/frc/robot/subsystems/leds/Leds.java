package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotState;

public class Leds extends SubsystemBase {

  private RobotState state = RobotState.getInstance();

  public int loopCycleCount = 0;
  public double autoFinishedTime = 0.0;

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView left;
  private final AddressableLEDBufferView right;
  private final Notifier loadingNotifier;

  private ShuffleboardTab ledTab = Shuffleboard.getTab(this.getName());
  private NetworkTableEntry enableTestEntry;
  private GenericEntry testBlinkEntry;
  private GenericEntry testBreatheEntry;
  private GenericEntry testScrollEntry;
  private SendableChooser<LedPatterns> testPattern;

  private NetworkTable ledTable;
  private NetworkTableEntry ledModeEntry;
  private NetworkTableEntry ledTestingEntry;

  private LEDPattern currentPattern = LedPatterns.BLACK.colorPatternOnly();
  private boolean enableTest = false;
  private boolean testBlink = false;
  private boolean testBreathe = false;
  private boolean testScroll = false;

  public Leds() {
    ledTable = NetworkTableInstance.getDefault().getTable("Leds");
    ledModeEntry = ledTable.getEntry("Mode");
    ledModeEntry.setString("OFF");

    ShuffleboardLayout ledTestingLayout =
        ledTab
            .getLayout("LED Testing Options", BuiltInLayouts.kGrid)
            .withSize(2, 3)
            .withPosition(0, 0);

    // enableTestEntry =
    //     ledTestingLayout
    //         .add("Enable LED Test", enableTest)
    //         .withWidget("Toggle Button")
    //         .withPosition(0, 0)
    //         .getEntry();
      
    // enableTestEntry

    testBlinkEntry =
        ledTestingLayout
            .add("Test Blink", testBlink)
            .withWidget("Toggle Button")
            .withPosition(1, 0)
            .getEntry();

    testBreatheEntry =
        ledTestingLayout
            .add("Test Breathe", testBreathe)
            .withWidget("Toggle Button")
            .withPosition(1, 1)
            .getEntry();

    testScrollEntry =
        ledTestingLayout
            .add("Test Scroll", testScroll)
            .withWidget("Toggle Button")
            .withPosition(1, 2)
            .getEntry();

    testPattern = new SendableChooser<LedPatterns>();
    for (LedPatterns pattern : LedPatterns.values()) {
      testPattern.addOption(pattern.toString(), pattern);
    }
    testPattern.setDefaultOption("BLACK", LedPatterns.BLACK);
    ledTestingLayout
        .add("Test Pattern", testPattern)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(0, 1);

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
              currentPattern = LedPatterns.RED.breathe();
              if (!Robot.isSimulation()) {
                currentPattern.applyTo(buffer);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public void periodic() {
    loopCycleCount += 1;
    if (loopCycleCount < LedConstants.MIN_LOOP_CYCLE_COUNT) {
      return;
    }
    loadingNotifier.stop();

    System.out.println(enableTestEntry.getBoolean(false));

    if (DriverStation.isTest() && enableTestEntry.getBoolean(false)) {
      try {
        LedPatterns pattern = LedPatterns.valueOf(ledModeEntry.getString("OFF"));
        testBlink = testBlinkEntry.getBoolean(false);
        testBreathe = testBreatheEntry.getBoolean(false);
        testScroll = testScrollEntry.getBoolean(false);
        if (testBlink) {
          currentPattern = pattern.blink();
        } else if (testBreathe) {
          currentPattern = pattern.breathe();
        } else if (testScroll) {
          currentPattern = pattern.scroll();
        } else {
          currentPattern = pattern.colorPatternOnly();
        }
      } catch (IllegalArgumentException E) {
        currentPattern = LedPatterns.RED.blink();
      }
    } else {
      ledModeEntry.setString(state.getMode().toString());
    }

    // Load the pattern onto the LEDs
    if (!Robot.isSimulation()) {
      currentPattern.applyTo(buffer);
      leds.setData(buffer);
    }
  }
}
