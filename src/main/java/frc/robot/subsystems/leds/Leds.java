package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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

  public enum Animations {
    NONE,
    BLINK,
    BREATHE,
    SCROLL
  }

  public int loopCycleCount = 0;
  public double autoFinishedTime = 0.0;

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView left;
  private final AddressableLEDBufferView right;
  private final Notifier loadingNotifier;

  private NetworkTableEntry enableTestEntry;
  private SendableChooser<LedPatterns> testPattern;
  private SendableChooser<Animations> testAnimation;
  private NetworkTable ledTable;

  private LEDPattern currentPattern = LedPatterns.BLACK.colorPatternOnly();
  private boolean enableTest = false;

  public Leds() {

    Shuffleboard.getTab("LEDs").add("LED Subsystem", this);



    // enableTestEntry =
    //     ledTestingLayout
    //         .add("Enable LED Test", enableTest)
    //         .withWidget("Toggle Button")
    //         .withPosition(0, 0)
    //         .getEntry();

    // enableTestEntry

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

    if (DriverStation.isTest()) {}

    if (DriverStation.isTest() && enableTestEntry.getBoolean(false)) {
      try {
        LedPatterns pattern = testPattern.getSelected();
        Animations animation = testAnimation.getSelected();
        switch (animation) {
          case BLINK -> currentPattern = pattern.blink();
          case BREATHE -> currentPattern = pattern.breathe();
          case SCROLL -> currentPattern = pattern.scroll();
          default -> currentPattern = pattern.colorPatternOnly();
        }
      } catch (IllegalArgumentException E) {
        currentPattern = LedPatterns.RED.blink();
      }
    } else {
      currentPattern = state.getMode().ledPattern;
    }

    // Load the pattern onto the LEDs
    if (!Robot.isSimulation()) {
      currentPattern.applyTo(buffer);
      leds.setData(buffer);
    } else {
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.publishConstString("LED Channel", "" + LedConstants.LED_CHANNEL);
    builder.addStringProperty(
        "Current Pattern",
        () -> currentPattern.toString(),
        (String x) -> {
          System.out.println(x);
        });
    builder.addBooleanProperty(
        "Enable Test",
        () -> enableTest,
        (boolean x) -> {
          enableTest = x;
        });

    enableTestEntry = NetworkTableInstance
      .getDefault()
        .getTable("SmartDashboard")
        .getEntry("Enable LED Test");
    enableTestEntry.setBoolean(false);

    this.initShuffleboardTab();

  }

  private void initShuffleboardTab() {

    ShuffleboardTab ledTab = Shuffleboard.getTab(this.getName());

    ShuffleboardLayout ledTestingLayout = ledTab
        .getLayout("LED Testing Options", BuiltInLayouts.kGrid)
        .withSize(2, 3)
        .withPosition(0, 0);

    testPattern = new SendableChooser<LedPatterns>();
    for (LedPatterns pattern : LedPatterns.values()) {
      testPattern.addOption(pattern.toString(), pattern);
    }
    testPattern.setDefaultOption("BLACK", LedPatterns.BLACK);
    ledTestingLayout
        .add("Test Pattern", testPattern)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 1);

    testAnimation = new SendableChooser<Animations>();
    for (Animations animation : Animations.values()) {
      testAnimation.addOption(animation.toString(), animation);
    }
    testAnimation.setDefaultOption("None", Animations.NONE);
    ledTestingLayout
        .add("Test Animation", testAnimation)
        .withWidget(BuiltInWidgets.kSplitButtonChooser)
        .withPosition(0, 2);

    ledTestingLayout.buildInto(null, null);

  }
  
}
  // private SendableChooser<LedPatterns> testPattern;
  // private SendableChooser<Animations> testAnimation;
