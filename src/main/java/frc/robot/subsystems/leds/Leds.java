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
    SCROLL
  }

  public enum Sections {
    FULL,
    LEFT,
    MIDDLE,
    RIGHT
  }

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView left;
  private final AddressableLEDBufferView right;
  private final AddressableLEDBufferView middle;

  private GenericEntry enableTestEntry;
  private SendableChooser<LedPatterns> testPattern;
  private SendableChooser<Animations> testAnimation;
  private SendableChooser<Sections> testSection;
  private GenericEntry testReverse;

  private LEDPattern currentPattern = LedPatterns.BLACK.colorPatternOnly();
  Sections applySection = Sections.FULL;
  Boolean isReversed = false;

  public Leds() {

    buffer = new AddressableLEDBuffer(LedConstants.LENGTH);
    left = buffer.createView(0, LedConstants.LENGTH / 2 - 1);
    middle = buffer.createView(LedConstants.LENGTH / 3, LedConstants.LENGTH * 2 / 3 - 1);
    right = buffer.createView(LedConstants.LENGTH / 2, LedConstants.LENGTH - 1);

    leds = new AddressableLED(LedConstants.LED_CHANNEL);
    leds.setLength(LedConstants.LENGTH);
    leds.setData(buffer);
    leds.start();

    initLedTabInShuffleboard();
  }

  @Override
  public void periodic() {

    if (enableTestEntry.getBoolean(false)) {
      try {
        LedPatterns pattern = testPattern.getSelected();
        Animations animation = testAnimation.getSelected();
        applySection = testSection.getSelected();
        isReversed = testReverse.getBoolean(false);

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
    LedPatterns.BLACK.colorPatternOnly().applyTo(buffer);
    if (!Robot.isSimulation()) {
      switch (applySection) {
        case FULL -> {
          if (isReversed) {
            // currentPattern.applyTo(buffer.reversed());
            currentPattern.applyTo(left.reversed());
            currentPattern.applyTo(right.reversed());
          } else {
            currentPattern.applyTo(buffer);
          }
        }

        case LEFT -> {
          if (isReversed) {
            currentPattern.applyTo(left.reversed());
          } else {
            currentPattern.applyTo(left);
          }
        }
        case MIDDLE -> {
          if (isReversed) {
            currentPattern.applyTo(middle.reversed());
          } else {
            currentPattern.applyTo(middle);
          }
        }
        case RIGHT -> {
          if (isReversed) {
            currentPattern.applyTo(right.reversed());
          } else {
            currentPattern.applyTo(right);
          }
        }
      }
      leds.setData(buffer);
    } else {
      // no op for simulation
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
  }
}
