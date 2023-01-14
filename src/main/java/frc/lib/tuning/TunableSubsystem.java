package frc.lib.tuning;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

/**
 * A subsystem that is tunable.
 *
 * @deprecated rework for 2023 and logging
 */
@Deprecated(since = "2023.1.1", forRemoval = false)
public abstract class TunableSubsystem extends SubsystemBase implements Tuner {

  private HashMap<String, GenericEntry> entries = new HashMap<>();

  /**
   * Creates a new SubsystemTuner.
   *
   * <p>Registers the tuner with the TunerManager, and initializes SubsystemBase.
   */
  protected TunableSubsystem() {
    super();

    TunerManager.getTunerManager().registerTuner(this);
  }

  @Override
  public String getTunerName() {
    return this.getName() + " Tuner";
  }

  @Override
  public Subsystem[] getTunerSubsystems() {
    return new Subsystem[] {this};
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
