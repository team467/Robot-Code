package frc.robot.subsystems.leds;

import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.Mode;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLED.ColorOrder;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.AddressableLEDBufferView;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.networktables.GenericEntry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

public class Leds implements Mechanism {

  public static final AddressableLEDBuffer buffer =
      new AddressableLEDBuffer(LedConstants.FULL_LENGTH);
  private static final boolean debug = false;
  private final AddressableLED leds;
  private final RobotState state = RobotState.getInstance();
  private final Selectable<LedPatterns> testPattern = new Selectable<>();
  private final Selectable<LedPatterns> testPattern2 = new Selectable<>();
  private final Selectable<Animations> testAnimation = new Selectable<>();
  private final Selectable<Sections> testSection = new Selectable<>();
  private final Selectable<Mode> testMode = new Selectable<>();
  private final Boolean isReversed = false;
  // TODO: Figure out how to properly setup Elastic
  private GenericEntry enableTestEntry;
  private GenericEntry testReverse;
  private GenericEntry testTimer;
  private GenericEntry enableLedModeTestEntry;
  private LEDPattern currentPattern = LedPatterns.BLACK.colorPatternOnly();
  private Sections applySection = Sections.FULL;

  public Leds() {
    leds = new AddressableLED(LedConstants.LED_CHANNEL);
    leds.setLength(LedConstants.FULL_LENGTH);
    leds.setColorOrder(ColorOrder.RGB);
    leds.setData(buffer);
    leds.setStart(0);

    for (LedPatterns pattern : LedPatterns.values()) {
      testPattern.add(pattern.name(), pattern);
      testPattern2.add(pattern.name(), pattern);
    }

    testPattern.setDefault("BLACK");
    testPattern2.setDefault("BLACK");

    for (Animations animation : Animations.values()) {
      testAnimation.add(animation.name(), animation);
    }
    testAnimation.setDefault("NONE");

    for (Sections section : Sections.values()) {
      testSection.add(section.name(), section);
    }
    testSection.setDefault("FULL");

    for (Mode mode : Mode.values()) {
      testMode.add(mode.name(), mode);
    }
    testMode.setDefault("OFF");

    Tunables.publish("LEDs/Testing/Pattern", testPattern);
    Tunables.publish("LEDs/Testing/Pattern2", testPattern2);
    Tunables.publish("LEDs/Testing/Animation", testAnimation);
    Tunables.publish("LEDs/Testing/Section", testSection);
    Tunables.publish("LEDs/Modes/Mode", testMode);

    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  public void periodic() {
//    if (enableTestEntry.getBoolean(false)) {
//      processLedPatternTestInputs();
//    } else if (enableLedModeTestEntry.getBoolean(false)) {
//      processLedModeTestInputs();
//    } else {
//      /* get from robot state  */
//      Mode ledMode = state.getMode();
//      currentPattern = ledMode.ledPattern;
//      applySection = ledMode.ledSection;
//    }

    // Load the pattern onto the LEDs
    if (!Robot.isSimulation()) {
      loadLedPatterns();
      leds.setData(buffer);
    }
    // else no op for simulation

  }

  // tests led patterns from shuffleboard gui
  private void processLedPatternTestInputs() {
    try {
      LedPatterns pattern = testPattern.getSelected();
      LedPatterns pattern2 = testPattern2.getSelected();
      Animations animation = testAnimation.getSelected();
//      double timer = testTimer.getDouble(0);
      applySection = testSection.getSelected();
//      isReversed = testReverse.getBoolean(false);

      switch (animation) {
//        case BLINK -> currentPattern = pattern.blink(timer);
//        case BREATHE -> currentPattern = pattern.breathe(timer);
//        case SCROLL -> currentPattern = pattern.scroll(timer);
        case BLEND -> currentPattern = pattern.blend(pattern2.colorPatternOnly());
        case OVERLAY -> currentPattern = pattern.overlayOn(pattern2.colorPatternOnly());
        case BREATHE_BLEND -> currentPattern = pattern.breathe().blend(pattern2.breathe(1.5));
        case BREATHE_OVERLAY -> currentPattern = pattern.breathe().overlayOn(pattern2.breathe());
        default -> currentPattern = pattern.colorPatternOnly();
      }
    } catch (IllegalArgumentException E) {
      currentPattern = LedPatterns.RED.blink();
    }
  }

  // process test inputs for led mode based on robot state
  private void processLedModeTestInputs() {
    try {
      Mode mode = testMode.getSelected();
      applySection = mode.ledSection;
      currentPattern = mode.ledPattern;
    } catch (IllegalArgumentException E) {
      currentPattern = LedPatterns.RED.blink();
    }
  }

  private void loadLedPatterns() {
    LedPatterns.BLACK.colorPatternOnly().applyTo(buffer);
    if (isReversed) {
      currentPattern.applyTo(applySection.getBufferView_1().reversed());
    } else {
      currentPattern.applyTo(applySection.getBufferView_1());
    }
    if (applySection.getBufferView_2() != null) {
      currentPattern.applyTo(applySection.getBufferView_2().reversed());
    }
  }

  public enum Animations {
    NONE,
    BLINK,
    BREATHE,
    SCROLL,
    BLEND,
    OVERLAY,
    BREATHE_BLEND,
    BREATHE_OVERLAY
  }

  public enum Sections {
    FULL(
        0,
        LedConstants.FULL_LENGTH / 2 - 1,
        LedConstants.FULL_LENGTH / 2,
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

    Sections(int start_1, int end_1, int start_2, int end_2) {
      if (debug) {
        System.out.println(
            "create section "
                + this
                + " start_1: "
                + start_1
                + " end_1: "
                + end_1
                + " start_2: "
                + start_2
                + " end_2:"
                + end_2);
      }
      this.buf_view_1 = buffer.createView(start_1, end_1);
      this.buf_view_2 = buffer.createView(start_2, end_2);
    }

    Sections(int start_1, int end_1) {
      if (debug) {
        System.out.println(
            "create section " + this + " start_1: " + start_1 + " end_1: " + end_1);
      }
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
}
