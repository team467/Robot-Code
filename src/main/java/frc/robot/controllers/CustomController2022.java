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

    public JoystickButton getButton(Buttons button) {
        return getButton(button.value);
    }

    public CustomController2022(int port) {
        super(port);
        addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x04, (byte) 0x06, (byte) 0x80});
        addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x03, (byte) 0x04, (byte) 0x03, (byte) 0x03});
        addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x03, (byte) 0x05, (byte) 0x03, (byte) 0x03});

        addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x03, (byte) 0x05, (byte) 0x03, (byte) 0x00});
        for (int i = 30; i < 40; i++) {
            addCommandToQueue(new byte[]{(byte) 0x06, (byte) 0x01, (byte) i, (byte) 0x01, (byte) 0x80, (byte) 0xFF, 0x00});

        }

    }
}
