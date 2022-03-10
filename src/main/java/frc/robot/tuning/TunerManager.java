package frc.robot.tuning;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.ArrayList;

public class TunerManager {
  private static TunerManager instance = null;
  private final ArrayList<Tuner> tuners = new ArrayList<>();
  private final SendableChooser<Tuner> tunerChooser = new SendableChooser<>();

  private TunerManager() {
    // no constructor needed
  }

  /**
   * @return the current tuner manager
   */
  public static synchronized TunerManager getTunerManager() {
    if (instance == null) {
      instance = new TunerManager();
    }

    return instance;
  }

  /**
   * Registers a new tuner
   *
   * @param tuner a tuner to be registered
   */
  public void registerTuner(Tuner tuner) {
    // System.out.println("Registered tuner: " + tuner.getTunerName());
    Shuffleboard.getTab(tuner.getTunerName())
        .add("Tuner Selection", tunerChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2, 1)
        .withPosition(0, 0);
    tuners.add(tuner);
    tuner.initializeTunerNetworkTables(Shuffleboard.getTab(tuner.getTunerName()));
    tunerChooser.addOption(tuner.getTunerName(), tuner);
  }

  public Tuner getTunerChoice() {
    return tunerChooser.getSelected();
  }
}
