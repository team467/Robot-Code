package frc.lib.input;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.ArrayList;

public abstract class CustomControllerBase extends Joystick {

  private final NetworkTable controllerTable =
      NetworkTableInstance.getDefault().getTable("Controller");

  private final RawPublisher commandQueueEntry =
      controllerTable.getRawTopic("CommandQueue").publish("raw");
  private final DoubleArrayPublisher commandQueueLengthEntry =
      controllerTable.getDoubleArrayTopic("CommandQueueLength").publish();
  private final BooleanPublisher hasCommandEntry =
      controllerTable.getBooleanTopic("HasCommand").publish(); // bool

  private final ArrayList<byte[]> commandList = new ArrayList<>();

  protected CustomControllerBase(int port) {
    super(port);

    commandQueueEntry.set(new byte[0]);
    commandQueueLengthEntry.set(new double[0]);
    hasCommandEntry.set(false);
  }

  /**
   * Check if the custom controller has a command.
   *
   * @return does the controller have a command
   */
  public boolean hasCommand() {
    return controllerTable.getBooleanTopic("HasCommand").getEntry(false).get();
  }

  /**
   * Adds a given command to the queue.
   *
   * @param command a command to add to queue
   * @see ControllerCommandFactory
   */
  public void addCommandToQueue(byte[] command) {
    commandList.add(command);
  }

  /** Updates the command queue. */
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

      commandQueueEntry.set(commandQueue);
      commandQueueLengthEntry.set(queueLength);
      hasCommandEntry.set(true);
      commandList.clear();
    }
  }

  /**
   * Gets a JoystickButton from a given button id.
   *
   * @param button the button id
   * @return a JoystickButton
   */
  public JoystickButton getButton(int button) {
    return new JoystickButton(this, button);
  }
}
