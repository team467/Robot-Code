package frc.lib.tuning;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.ArrayList;

/**
 * A class that manages all active tuners and shows them in shuffleboard.
 *
 * @deprecated rework for 2023 and logging
 */
@Deprecated(since = "2023.1.1", forRemoval = false)
public class TunerManager {

  private static TunerManager instance = null;
  private final ArrayList<Tuner> tuners = new ArrayList<>();
  private final SendableChooser<Tuner> tunerChooser = new SendableChooser<>();

  private TunerManager() {}

  /**
   * Register a tuner with the TunerManager.
   *
   * @param tuner The tuner to register.
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
    tunerChooser.setDefaultOption(tuner.getTunerName(), tuner);
  }

  /**
   * Get the tuner that is currently selected.
   *
   * @return The tuner that is currently selected.
   */
  public Tuner getTunerChoice() {
    return tunerChooser.getSelected();
  }

  public static synchronized TunerManager getTunerManager() {
    if (instance == null) {
      instance = new TunerManager();
    }

    return instance;
  }
}
