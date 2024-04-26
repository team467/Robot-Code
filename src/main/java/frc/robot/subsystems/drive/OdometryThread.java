package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for devices like the SparkMax that require polling rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent timing.
 */
public class OdometryThread {
  private final List<Supplier<OptionalDouble>> signals = new ArrayList<>();
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier;
  private static OdometryThread instance = null;

  /**
   * Get OdometryThread singleton
   * @return OdometryThread object
   */
  public static OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }
    return instance;
  }

  /**
   * Initializes the notifier and sets its name.
   */
  private OdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("SparkMaxOdometryThread");
  }

  /**
   * Starts the periodic execution of the notifier if the timestamp queues are not empty.
   */
  public void start() {
    if (!timestampQueues.isEmpty()) {
      notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQUENCY); // hz to seconds
    }
  }

  /**
   * Registers a signal supplier and creates a queue to store the signal values.
   *
   * @param signal The supplier that provides the signal values.
   * @return The queue that stores the signal values.
   */
  public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /**
   * Creates a new queue for storing timestamps.
   *
   * @return A queue for storing timestamps.
   */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /**
   * This method is called periodically by a notifier thread in the {@link OdometryThread} class.
   * It is responsible for gathering high-frequency measurements from signal suppliers,
   * such as odometry sensors, and storing them in corresponding queues for further processing.
   *
   * @see OdometryThread#start()
   */
  private void periodic() {
    Drive.odometryLock.lock();

    // Converts timestamp into milliseconds
    double timestamp = Logger.getRealTimestamp() / 1e6;
    try {
      double[] values = new double[signals.size()];
      boolean isValid = true;
      for (int i = 0; i < signals.size(); i++) {
        OptionalDouble value = signals.get(i).get();
        if (value.isPresent()) {
          values[i] = value.getAsDouble();
        } else {
          isValid = false;
          break;
        }
      }
      if (isValid) {
        // Add each individual signal reading to the appropriate data queue
        for (int i = 0; i < queues.size(); i++) {
          queues.get(i).offer(values[i]);
        }

        // Add the timestamp at which each reading was taken to the appropriate timestamp queue
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      }
    } finally {
      Drive.odometryLock.unlock();
    }
  }
}
