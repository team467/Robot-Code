package frc.robot.tuning;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunerManager {
    TunerManager instance = null;
    ArrayList<Tuner> tuners = new ArrayList<>();
    SendableChooser<Tuner> tunerChooser = new SendableChooser<>();

    private TunerManager() {}

    public void registerTuner(Tuner tuner) {
        tuners.add(tuner);
        tunerChooser.addOption(tuner.getTunerName(), tuner);
    }

    public void addChooserToNT() {
        SmartDashboard.putData(tunerChooser);
    }

    public Tuner getTunerChoice() {
        return tunerChooser.getSelected();
    }

    public TunerManager getTunerManager() {
        if (instance == null) {
            instance = new TunerManager();
        }

        return instance;
    }
}
