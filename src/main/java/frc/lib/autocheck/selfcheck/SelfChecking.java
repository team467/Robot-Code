package frc.lib.autocheck.selfcheck;

import frc.lib.autocheck.SubsystemFault;

import java.util.List;

/**
 * The SelfChecking interface defines a contract for classes that can perform self-checking operations
 * to identify and report any faults in the subsystems.
 */
public interface SelfChecking {
  /**
   * Checks for faults in the subsystem.
   *
   * @return A list of SubsystemFault objects representing the faults found.
   */
  List<SubsystemFault> checkForFaults();
}
