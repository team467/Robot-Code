package frc.lib.input;

import java.util.ArrayList;

public class ControllerQueue {

  private static ControllerQueue INSTANCE = null;

  private final ArrayList<CustomControllerBase> controllers = new ArrayList<>();

  /**
   * Gets the instance of the controller queue.
   *
   * @return the controller queue
   */
  public static ControllerQueue getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ControllerQueue();
    }

    return INSTANCE;
  }

  /**
   * @see #getInstance()
   */
  private ControllerQueue() {}

  /**
   * Adds a new custom controller to the queue.
   *
   * @param controller a custom controller
   */
  public void addController(CustomControllerBase controller) {
    controllers.add(controller);
  }

  /** Updates the queue of all controllers. */
  @SuppressWarnings("Convert2MethodRef")
  public void run() {
    controllers.forEach(CustomControllerBase::updateQueue);
  }

  /**
   * Gets a controller from the controller list.
   *
   * @param index the controller id
   * @return the controller
   */
  public CustomControllerBase get(int index) {
    return controllers.get(index);
  }
}
