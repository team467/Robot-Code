package frc.robot.tuning;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class CompositeTuner implements Tuner {
    ArrayList<Subsystem> subsystems = new ArrayList<>();
    private HashMap<String, NetworkTableEntry> entries = new HashMap<>();

    public CompositeTuner() {
        super();

        TunerManager.getTunerManager().registerTuner(this);
    }

    abstract public String getName();

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
