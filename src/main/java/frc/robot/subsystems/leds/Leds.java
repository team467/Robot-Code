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
  private final AddressableLED leds;
  public static AddressableLEDBuffer buffer = new AddressableLEDBuffer(LedConstants.FULL_LENGTH);

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
    FULL(
        0,
        LedConstants.FULL_LENGTH * 1 / 2 - 1,
        LedConstants.FULL_LENGTH * 1 / 2,
        LedConstants.FULL_LENGTH - 1),
    BASE1(LedConstants.BASE1_START, LedConstants.BASE1_END),
    BASE2(LedConstants.BASE2_START, LedConstants.BASE2_END),
    BASE1_BASE2(
        LedConstants.BASE1_START,
        LedConstants.BASE1_END,
        LedConstants.BASE2_START,
        LedConstants.BASE2_END),
    BAR(LedConstants.BAR_START, LedConstants.BAR_END),

    FIRST_QUARTER(
        LedConstants.BASE1_FIRST_QUARTER_START,
        LedConstants.BASE1_FIRST_QUARTER_END,
        LedConstants.BASE2_FIRST_QUARTER_START,
        LedConstants.BASE2_FIRST_QUARTER_END),
    SECOND_QUARTER(
        LedConstants.BASE1_SECOND_QUARTER_START,
        LedConstants.BASE1_SECOND_QUARTER_END,
        LedConstants.BASE2_SECOND_QUARTER_START,
        LedConstants.BASE2_SECOND_QUARTER_END),
    THIRD_QUARTER(
        LedConstants.BASE1_THIRD_QUARTER_START,
        LedConstants.BASE1_THIRD_QUARTER_END,
        LedConstants.BASE2_THIRD_QUARTER_START,
        LedConstants.BASE2_THIRD_QUARTER_END),
    FOURTH_QUARTER(
        LedConstants.BASE1_FOURTH_QUARTER_START,
        LedConstants.BASE1_FOURTH_QUARTER_END,
        LedConstants.BASE2_FOURTH_QUARTER_START,
        LedConstants.BASE2_FOURTH_QUARTER_END);

    private final AddressableLEDBufferView buf_view_1;
    private final AddressableLEDBufferView buf_view_2;

    private Sections(int start_1, int end_1, int start_2, int end_2) {
      System.out.println(
          "create section"
              + "start_1"
              + start_1
              + "end_1"
              + end_1
              + "start_2"
              + start_2
              + "end_2"
              + end_2);
      this.buf_view_1 = buffer.createView(start_1, end_1);
      this.buf_view_2 = buffer.createView(start_2, end_2);
    }

    private Sections(int start_1, int end_1) {
      System.out.println("create section" + "start_1" + start_1 + "end_1" + end_1);
      this.buf_view_1 = buffer.createView(start_1, end_1);
      this.buf_view_2 = null;
    }

    public AddressableLEDBufferView getBufferView_1() {
      return buf_view_1;
    }

    public AddressableLEDBufferView getBufferView_2() {
      return buf_view_2;
    }
  }

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

    leds = new AddressableLED(LedConstants.LED_CHANNEL);
    leds.setLength(LedConstants.FULL_LENGTH);
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
      applySection = state.getMode().ledSection;
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
    if (isReversed) {
      currentPattern.applyTo(applySection.getBufferView_1().reversed());
      if (applySection.getBufferView_2() != null) {
        currentPattern.applyTo(applySection.getBufferView_2().reversed());
      }
    } else {
      currentPattern.applyTo(applySection.getBufferView_1());
      if (applySection.getBufferView_2() != null) {
        currentPattern.applyTo(applySection.getBufferView_2().reversed());
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
