package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

public abstract class CustomControllerBase extends Joystick {
    private NetworkTable controllerTable = NetworkTableInstance.getDefault().getTable("controller");

    private NetworkTableEntry robotConnectedEntry = controllerTable.getEntry("robotConnected"); // bool
    private NetworkTableEntry clientConnectedEntry = controllerTable.getEntry("clientConnected"); // bool

    private NetworkTableEntry commandEntry = controllerTable.getEntry("command"); // raw bytes
    private NetworkTableEntry hasCommandEntry = controllerTable.getEntry("hasCommand"); // bool
    private NetworkTableEntry responseEntry = controllerTable.getEntry("response"); // raw bytes
    private NetworkTableEntry hasResponseEntry = controllerTable.getEntry("hasResponse"); // bool

    public CustomControllerBase(int port) {
        super(port);
        
        robotConnectedEntry.setBoolean(true);
        clientConnectedEntry.setBoolean(false);

        commandEntry.setRaw(new byte[0]);
        hasCommandEntry.setBoolean(false);
        responseEntry.setRaw(new byte[0]);
        hasResponseEntry.setBoolean(false);
    }

    public boolean isRobotConnected() {
        return robotConnectedEntry.getBoolean(false);
    }

    public boolean isClientConnected() {
        return clientConnectedEntry.getBoolean(false);
    }

    public void sendCommand(byte[] command) {
        commandEntry.setRaw(command);
        hasCommandEntry.setBoolean(true);
    }

    public boolean hasCommand() {
        return hasCommandEntry.getBoolean(false);
    }


    public boolean hasReponse() {
        return hasResponseEntry.getBoolean(false);
    }

    public byte[] getResponse() {
        return hasResponseEntry.getRaw(new byte[0]);
    }
}
