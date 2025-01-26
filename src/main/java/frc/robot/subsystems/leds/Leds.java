package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
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

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView left;
  private final AddressableLEDBufferView right;

  private GenericEntry enableTestEntry;
  private SendableChooser<LedPatterns> testPattern;
  private SendableChooser<Animations> testAnimation;
  private NetworkTable ledTable;

  private LEDPattern currentPattern = LedPatterns.BLACK.colorPatternOnly();

  public Leds() {

    buffer = new AddressableLEDBuffer(LedConstants.LENGTH);
    left = buffer.createView(0, LedConstants.LENGTH / 2 - 1);
    right = buffer.createView(LedConstants.LENGTH / 2, LedConstants.LENGTH - 1);

    leds = new AddressableLED(LedConstants.LED_CHANNEL);
    leds.setLength(LedConstants.LENGTH);
    leds.setData(buffer);
    leds.start();

    initLedTabInShuffleboard();
  }

  @Override
  public void periodic() {

    System.out.println("Entry " + enableTestEntry.get() + " " + enableTestEntry.getBoolean(false));

    if (enableTestEntry.getBoolean(false)) {
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

  private void initLedTabInShuffleboard() {

    ShuffleboardTab ledTab = Shuffleboard.getTab("LEDs");
    ShuffleboardLayout ledTestingLayout =
        ledTab
            .getLayout("LED Testing Options", BuiltInLayouts.kGrid)
            .withSize(2, 4)
            .withPosition(0, 0);

    enableTestEntry =
        ledTestingLayout
            .add("Enable Test", "boolean", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .getEntry();

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
  }
}
  // private SendableChooser<LedPatterns> testPattern;
  // private SendableChooser<Animations> testAnimation;
