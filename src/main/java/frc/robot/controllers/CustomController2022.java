package frc.robot.controllers;

public class CustomController2022 extends CustomControllerBase {
    enum Buttons {
        A
    }

    public CustomController2022(int port) {
        super(port);

        for (byte i = 0; i < 6; i++) {
            addCommandToQueue(new byte[]{0x06, 0x01, i, 0x01, (byte) (i * 20), (byte) (i * 10), 0x05});
        }
        
    }
}
