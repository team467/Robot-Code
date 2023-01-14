package frc.lib.tuning;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * @deprecated rework for 2023 and logging
 */
@Deprecated(since = "2023.1.1", forRemoval = false)
public abstract class CompositeTuner implements Tuner {

  ArrayList<Subsystem> subsystems = new ArrayList<>();
  private HashMap<String, GenericEntry> entries = new HashMap<>();

  /** Creates a new CompositeTuner. */
  protected CompositeTuner() {
    TunerManager.getTunerManager().registerTuner(this);
  }

  /**
   * Gets the name of the composite tuner.
   *
   * @return The name of the composite tuner.
   */
  public abstract String getName();

  @Override
  public String getTunerName() {
    return this.getName() + " Tuner";
  }

  @Override
  public Subsystem[] getTunerSubsystems() {
    Subsystem[] array = new Subsystem[subsystems.size()];
    return subsystems.toArray(array);
  }

  /**
   * Adds a subsystem to the composite tuner.
   *
   * @param subsystem The subsystem to add.
   */
  public void addSubsystem(Subsystem subsystem) {
    subsystems.add(subsystem);
  }

  @Override
  public void addEntry(String name, GenericEntry entry) {
    entries.put(name, entry);
  }

  @Override
  public GenericEntry getEntry(String name) {
    return entries.get(name);
  }
}
