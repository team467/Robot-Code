package frc.robot.input;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.input.ControllerCommandFactory;
import frc.lib.input.ControllerDataProtocol;
import frc.lib.input.CustomControllerBase;

public class CustomController2022 extends CustomControllerBase {
  public enum Buttons {
    FLUSH(1),
    SHOOT(2),
    CLIMBER_LIMITS(3),
    SHOOTER_AUTO(4),
    EVERYTHING(5),
    CLIMBER_LOCK(6),
    CLIMBER_UP(7),
    CLIMBER_DOWN(8);

    public final int value;

    Buttons(int value) {
      this.value = value;
    }
  }

  public enum Sections {
    UNDER_BOTTOM(0),
    UNDER_RIGHT(1),
    UNDER_TOP(2),
    UNDER_LEFT(3),
    RING_TOP(4),
    RING_BOTTOM(5),
    RING(6),
    UNDER(7);

    public final int value;

    Sections(int value) {
      this.value = value;
    }
  }

  public JoystickButton getButton(Buttons button) {
    return getButton(button.value);
  }

  public CustomController2022(int port) {
    super(port);
    addCommandToQueue(ControllerCommandFactory.setAllLEDBrightness(0x80));
    addCommandToQueue(
        ControllerCommandFactory.setAllLEDEffect(ControllerDataProtocol.LightingEffect.Static));
    addCommandToQueue(
        ControllerCommandFactory.setLEDSectionBaseColor(Sections.RING_TOP.value, Color.kRed));
    addCommandToQueue(
        ControllerCommandFactory.setLEDSectionBaseColor(Sections.RING_BOTTOM.value, Color.kBlue));
    addCommandToQueue(
        ControllerCommandFactory.setLEDSectionEffectSpaced(
            Sections.UNDER.value, ControllerDataProtocol.LightingEffect.ColorCycle));
  }
}
