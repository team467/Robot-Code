package frc.lib.tuning;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * @deprecated rework for 2023 and logging
 */
@Deprecated(since = "2023.1.1", forRemoval = false)
public interface Tuner {

  /**
   * Gets the name of the tuner.
   *
   * @return The name of the tuner.
   */
  String getTunerName();

  /**
   * Gets the subsystems that the tuner is for.
   *
   * @return The subsystems that the tuner is for.
   */
  Subsystem[] getTunerSubsystems();

  /**
   * Initializes the tuner network table.
   *
   * @param tab The shuffleboard tab to add the tuner to.
   */
  void initializeTunerNetworkTables(ShuffleboardTab tab);

  /** Initializes the tuner. */
  void initializeTuner();

  /**
   * Adds an entry to the tuner.
   *
   * @param name The name of the entry.
   * @param entry The entry to add.
   */
  void addEntry(String name, GenericEntry entry);

  /**
   * Gets an entry from the tuner.
   *
   * @param name The name of the entry.
   * @return The entry.
   */
  GenericEntry getEntry(String name);
}
