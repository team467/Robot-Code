package frc.lib.autocheck.selfcheck;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import frc.lib.autocheck.SubsystemFault;

import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPWMMotor implements SelfChecking {
  private final String label;
  private final PWMMotorController motor;

  /**
   * Creates a new instance of SelfCheckingPWMMotor.
   *
   * @param label    the label for the SelfCheckingPWMMotor
   * @param motor the PWMMotorController object
   */
  public SelfCheckingPWMMotor(String label, PWMMotorController motor) {
    this.label = label;
    this.motor = motor;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (!motor.isAlive()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device timed out", label)));
    }

    return faults;
  }
}
