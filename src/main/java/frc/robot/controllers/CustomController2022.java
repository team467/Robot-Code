package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

        public final byte value;
    
        Sections(byte value) {
            this.value = value;
        }

        Sections(int value) {
            this.value = (byte) value;
        }
    }

    public JoystickButton getButton(Buttons button) {
        return getButton(button.value);
    }

    public CustomController2022(int port) {
        super(port);
        addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x04, (byte) 0x06, (byte) 0x80});
        addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x03, Sections.RING.value, (byte) 0x03, (byte) 0x03});
        addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x03, Sections.UNDER.value, (byte) 0x03, (byte) 0x03});
    }
}
