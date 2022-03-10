package frc.robot.motors;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface MotorControllerEncoder extends MotorController {

  /**
   * Get the position of the motor.
   *
   * @return the position of the motor
   */
  double getPosition();

  /**
   * Gets the velocity of the motor in revs per second.
   *
   * @return the velocity of the motor
   */
  double getVelocity();

  /** Sets the sensor position to 0 */
  void resetPosition();

  /**
   * Gets current output in amps.
   *
   * @return current output in amps
   */
  double getCurrent();

  /**
   * Sets the units per rotation.
   *
   * @param unitsPerRotation the units per rotation.
   */
  void setUnitsPerRotation(double unitsPerRotation);
}
