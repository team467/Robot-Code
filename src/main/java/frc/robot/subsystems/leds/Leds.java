package frc.robot.subsystems.leds;

import edu.wpi.first.networktables.GenericEntry;
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
    SCROLL,
    BLEND,
    OVERLAY,
    BREATHE_BLEND,
    BREATHE_OVERLAY;
  }

  public enum Sections {
    FULL,
    FIRST,
    MIDDLE_1,
    MIDDLE_2,
    LAST
  }

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView first;
  private final AddressableLEDBufferView middle1;
  private final AddressableLEDBufferView middle2;
  private final AddressableLEDBufferView last;

  private GenericEntry enableTestEntry;
  private SendableChooser<LedPatterns> testPattern;
  private SendableChooser<LedPatterns> testPattern2;
  private SendableChooser<Animations> testAnimation;
  private SendableChooser<Sections> testSection;
  private GenericEntry testReverse;
  private GenericEntry testTimer;

  private LEDPattern currentPattern = LedPatterns.BLACK.colorPatternOnly();
  private Sections applySection = Sections.FULL;
  private Boolean isReversed = false;

  public Leds() {

    buffer = new AddressableLEDBuffer(LedConstants.LENGTH);
    first = buffer.createView(0, LedConstants.LENGTH / 4 - 1);
    middle1 = buffer.createView(LedConstants.LENGTH / 4, LedConstants.LENGTH / 2 - 1);
    middle2 = buffer.createView(LedConstants.LENGTH / 2, LedConstants.LENGTH * 3 / 4 - 1);
    last = buffer.createView(LedConstants.LENGTH * 3 / 4, LedConstants.LENGTH - 1);

    leds = new AddressableLED(LedConstants.LED_CHANNEL);
    leds.setLength(LedConstants.LENGTH);
    leds.setData(buffer);
    leds.start();

    initLedTabInShuffleboard();
  }

  @Override
  public void periodic() {

    if (enableTestEntry.getBoolean(false)) {
      processTestInputs();
    } else {
      /* get from robot state  */
      currentPattern = state.getMode().ledPattern;
      /*
      applySection = state.getMode().ledSection;
      isReversed = state.getMode().isReversed;
      */
    }

    // Load the pattern onto the LEDs
    if (!Robot.isSimulation()) {
      loadLedPatterns();
      leds.setData(buffer);
    } else {
      // no op for simulation
    }
  }

  private void processTestInputs() {
    try {
      LedPatterns pattern = testPattern.getSelected();
      LedPatterns pattern2 = testPattern2.getSelected();
      Animations animation = testAnimation.getSelected();
      double timer = testTimer.getDouble(0);
      applySection = testSection.getSelected();
      isReversed = testReverse.getBoolean(false);
      System.out.println("test timer" + timer);

      switch (animation) {
        case BLINK -> currentPattern = pattern.blink(timer);
        case BREATHE -> currentPattern = pattern.breathe(timer);
        case SCROLL -> currentPattern = pattern.scroll(timer);
        case BLEND -> currentPattern = pattern.blend(pattern2.colorPatternOnly());
        case OVERLAY -> currentPattern = pattern.overlayon(pattern2.colorPatternOnly());
        case BREATHE_BLEND -> currentPattern = pattern.breathe().blend(pattern2.breathe(1.5));
        case BREATHE_OVERLAY -> currentPattern = pattern.breathe().overlayOn(pattern2.breathe());
        default -> currentPattern = pattern.colorPatternOnly();
      }
    } catch (IllegalArgumentException E) {
      currentPattern = LedPatterns.RED.blink();
    }
  }

  private void loadLedPatterns() {
    LedPatterns.BLACK.colorPatternOnly().applyTo(buffer);
    switch (applySection) {
      case FULL -> {
        if (isReversed) {
          // currentPattern.applyTo(buffer.reversed());
          currentPattern.applyTo(first.reversed());
          currentPattern.applyTo(last.reversed());
        } else {
          currentPattern.applyTo(buffer);
        }
      }
      case FIRST -> {
        if (isReversed) {
          currentPattern.applyTo(first.reversed());
        } else {
          currentPattern.applyTo(first);
        }
      }
      case MIDDLE_1 -> {
        if (isReversed) {
          currentPattern.applyTo(middle1.reversed());
        } else {
          currentPattern.applyTo(middle1);
        }
      }
      case MIDDLE_2 -> {
        if (isReversed) {
          currentPattern.applyTo(middle2.reversed());
        } else {
          currentPattern.applyTo(middle2);
        }
      }
      case LAST -> {
        if (isReversed) {
          currentPattern.applyTo(last.reversed());
        } else {
          currentPattern.applyTo(last);
        }
      }
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

    testPattern2 = new SendableChooser<LedPatterns>();
    for (LedPatterns pattern : LedPatterns.values()) {
      testPattern2.addOption(pattern.toString(), pattern);
    }
    testPattern2.setDefaultOption("BLACK", LedPatterns.BLACK);
    ledTestingLayout
        .add("Test Pattern2", testPattern2)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 1);

    testAnimation = new SendableChooser<Animations>();
    for (Animations animation : Animations.values()) {
      testAnimation.addOption(animation.toString(), animation);
    }
    testAnimation.setDefaultOption("None", Animations.NONE);
    ledTestingLayout
        .add("Test Animation", testAnimation)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 2);

    testSection = new SendableChooser<Sections>();
    for (Sections section : Sections.values()) {
      testSection.addOption(section.toString(), section);
    }
    testSection.setDefaultOption("FULL", Sections.FULL);
    ledTestingLayout
        .add("Test Section", testSection)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 3);

    testReverse =
        ledTestingLayout
            .add("Test Reverse", "boolean", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .getEntry();

    testTimer =
        ledTestingLayout
            .add("Test Timer", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withPosition(0, 0)
            .getEntry();
  }
}
