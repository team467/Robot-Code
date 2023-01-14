package frc.lib.io.motorcontroller;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** An abstract IO for controlling a motor controller. */
public interface SimpleMotorControllerIO {

  class SimpleMotorControllerIOInputs implements LoggableInputs {

    /** The position of the motor based on if a conversion factor was provided */
    public double position = 0.0;
    /** The velocity of the motor based on if a conversion factor was provided */
    public double velocity = 0.0;
    /** The voltage applied to the motor */
    public double appliedVolts = 0.0;
    /** The current drawn by the motor */
    public double[] current = new double[] {};

    public void toLog(LogTable table) {
      table.put("Position", position);
      table.put("Velocity", velocity);
      table.put("AppliedVolts", appliedVolts);
      table.put("Current", current);
    }

    public void fromLog(LogTable table) {
      position = table.getDouble("Position", position);
      velocity = table.getDouble("Velocity", velocity);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      current = table.getDoubleArray("Current", current);
    }
  }

  /**
   * Update the inputs of the motor controller
   *
   * @param inputs The inputs to update
   */
  default void updateInputs(SimpleMotorControllerIOInputs inputs) {}

  /**
   * Set the speed of the motor controller
   *
   * @param speed The speed to set the motor controller to
   */
  default void setSpeed(double speed) {}

  /**
   * Set the voltage of the motor controller
   *
   * @param volts The voltage to set the motor controller to
   */
  default void setVoltage(double volts) {}
}
