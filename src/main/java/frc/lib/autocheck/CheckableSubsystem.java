package frc.lib.autocheck;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The CheckableSubsystem class is an abstract class that extends the SubsystemBase class. It is
 * responsible for registering a system check with the FaultReporter instance.
 *
 * <p>The CheckableSubsystem class is designed to be subclassed by specific subsystems that need to
 * perform a system check to verify the health and readiness of the subsystem.
 *
 * <p>Subclasses of CheckableSubsystem must provide an implementation for the systemCheckCommand()
 * method, which returns the system check command to register.
 *
 * @see SubsystemBase
 * @see FaultReporter
 */
public abstract class CheckableSubsystem extends SubsystemBase {
  /**
   * Constructs a new instance of the CheckableSubsystem class. The CheckableSubsystem class is
   * responsible for registering a system check with the FaultReporter instance.
   *
   * <p>This constructor initializes the CheckableSubsystem by calling the super constructor and
   * then registering the system check with the FaultReporter.
   *
   * <p>The system check is registered with the name of the subsystem and the system check command.
   * The system check is used to verify the health and readiness of the subsystem.
   *
   * @see FaultReporter
   */
  public CheckableSubsystem() {
    super();
    FaultReporter.getInstance().registerSystemCheck(this.getName(), systemCheckCommand());
  }

  /**
   * A system check command that ensures that all parts of the subsystem are working properly.
   *
   * @return the system check command to register
   */
  public abstract CommandBase systemCheckCommand();
}
