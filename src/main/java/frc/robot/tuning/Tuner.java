package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Tuner {

  /**
   * Gets the name of the current tuner
   *
   * @return the tuner's name
   */
  String getTunerName();

  /** TODO: add explanation as to what this is */
  Subsystem[] getTunerSubsystems();

  /**
   * Sets up the tuner's network tables using the Shuffleboard
   *
   * @param tab a ShuffleboardTab that will contain the network table adjustments
   */
  void initializeTunerNetworkTables(ShuffleboardTab tab);

  /** Sets the default values for the tuner */
  void initializeTuner();

  /**
   * Adds an entry to the tuner
   *
   * @param name the name of the entry
   * @param entry a network table entry
   */
  void addEntry(String name, NetworkTableEntry entry);

  /**
   * Gets an entry from the tuner
   *
   * @param name the name of the entry
   * @return the network table's entry
   */
  NetworkTableEntry getEntry(String name);
}
