package frc.lib.autocheck.selfcheck;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import frc.lib.autocheck.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingSparkMax implements SelfChecking {
  private final String label;
  private CANSparkMax spark;

  /**
   * Creates a new instance of SelfCheckingSparkMax.
   *
   * @param label the label for the SelfCheckingSparkMax
   * @param spark the CANSparkMax object
   */
  public SelfCheckingSparkMax(String label, CANSparkMax spark) {
    this.label = label;
    this.spark = spark;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    ArrayList<SubsystemFault> faults = new ArrayList<>();

    REVLibError err = spark.getLastError();
    if (err != REVLibError.kOk) {
      faults.add(new SubsystemFault(String.format("[%s]: Error: %s", label, err.name())));
    }

    return faults;
  }
}
