package frc.robot.controllers;

public class CustomController2022 extends CustomControllerBase {
    enum Buttons {
        A
    }

    public CustomController2022(int port) {
        super(port);
        sendCommand(new byte[]{0x06, 0x04, 0x03, 0x03});
    }
}
