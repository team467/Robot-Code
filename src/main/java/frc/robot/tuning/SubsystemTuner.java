package frc.robot.tuning;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemTuner extends SubsystemBase implements Tuner {
    HashMap<String, TunerParameter> tunerParameters = new HashMap<>();

    public SubsystemTuner() {
        super();

        TunerManager.getTunerManager().registerTuner(this);
    }

    @Override
    public String getTunerName() {
        return this.getName() + " Tuner";
    }

    @Override
    public Subsystem[] getTunerSubsystems() {
        return new Subsystem[]{this};
    }

    @Override
    public TunerParameter[] getTunerParameters() {
        TunerParameter[] tunerArray = new TunerParameter[tunerParameters.size()];
        tunerArray = tunerParameters.values().toArray(tunerArray);
        return tunerArray;
    }

    @Override
    public void addTunerParameter(String tunerName, TunerParameter tunerParameter) {
        tunerParameters.put(tunerName, tunerParameter);
    }

    @Override
    public TunerParameter getTunerParameter(String tunerName) {
        return tunerParameters.get(tunerName);
    }
}
