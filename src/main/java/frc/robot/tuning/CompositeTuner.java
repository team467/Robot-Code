package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.HashMap;

public abstract class CompositeTuner implements Tuner {
  ArrayList<Subsystem> subsystems = new ArrayList<>();
  private HashMap<String, NetworkTableEntry> entries = new HashMap<>();

  protected CompositeTuner() {
    super();

    TunerManager.getTunerManager().registerTuner(this);
  }

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

  public void addSubsystem(Subsystem subsystem) {
    subsystems.add(subsystem);
  }

  @Override
  public void addEntry(String name, NetworkTableEntry entry) {
    entries.put(name, entry);
  }

  @Override
  public NetworkTableEntry getEntry(String name) {
    return entries.get(name);
  }
}
