package frc.robot.controllers;

public class CustomController2022 extends CustomControllerBase {
    enum Buttons {
        A
    }

    public CustomController2022(int port) {
        super(port);
        // addCommandToQueue(new byte[]{0x06, 0x04, 0x03, 0x03});
        for (int i = 0; i < 6; i++) {
            addCommandToQueue(new byte[]{0x06, 0x01, (byte) i, 0x01, (byte) ((i + 1) * (255/7)), 20, 127});
        }
    }
}
