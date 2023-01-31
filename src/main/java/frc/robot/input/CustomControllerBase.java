package frc.robot.input;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.ArrayList;

public abstract class CustomControllerBase extends Joystick {
  private NetworkTable controllerTable = NetworkTableInstance.getDefault().getTable("Controller");

  private NetworkTableEntry commandQueueEntry =
      controllerTable.getEntry("CommandQueue"); // raw bytes
  private NetworkTableEntry commandQueueLengthEntry =
      controllerTable.getEntry("CommandQueueLength"); // double array
  private NetworkTableEntry hasCommandEntry = controllerTable.getEntry("HasCommand"); // bool

  private ArrayList<byte[]> commandList = new ArrayList<>();

  public CustomControllerBase(int port) {
    super(port);

    commandQueueEntry.setRaw(new byte[0]);
    commandQueueLengthEntry.setDoubleArray(new double[0]);
    hasCommandEntry.setBoolean(false);
  }

  public boolean hasCommand() {
    return hasCommandEntry.getBoolean(false);
  }

  public void addCommandToQueue(byte[] command) {
    commandList.add(command);
  }

  public void updateQueue() {
    if (!hasCommand() && !commandList.isEmpty()) {
      int totalQueueLength = 0;
      double[] queueLength = new double[commandList.size()];

      for (int i = 0; i < commandList.size(); i++) {
        queueLength[i] = commandList.get(i).length;
        totalQueueLength += commandList.get(i).length;
      }

      byte[] commandQueue = new byte[totalQueueLength];
      int commandPointer = 0;
      for (byte[] bytes : commandList) {
        for (byte b : bytes) {
          commandQueue[commandPointer] = b;
          commandPointer += 1;
        }
      }

      commandQueueEntry.setRaw(commandQueue);
      commandQueueLengthEntry.setDoubleArray(queueLength);
      hasCommandEntry.setBoolean(true);
      commandList.clear();
    }
  }

  public JoystickButton getButton(int button) {
    return new JoystickButton(this, button);
  }
}
