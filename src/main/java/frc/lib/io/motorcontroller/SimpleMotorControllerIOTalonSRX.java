package frc.lib.io.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.util.Units;
import frc.lib.autocheck.FaultReporter;

/** A Motor Controller IO class that interacts with a TalonSRX. */
public class SimpleMotorControllerIOTalonSRX implements SimpleMotorControllerIO {

  private final WPI_TalonSRX motor;
  private double rotsToRads = 1.0;

  /**
   * Create a new SimpleMotorControllerIOTalonSRX
   *
   * @param motorId The ID of the motor
   * @param inverted Whether the motor is inverted
   * @param gearRatio The gear ratio in inputs/outputs
   * @param subsystemName The name of the subsystem using the motor controller, for use with the
   *     self-check tool
   */
  public SimpleMotorControllerIOTalonSRX(
      int motorId, boolean inverted, double gearRatio, String subsystemName) {
    motor = new WPI_TalonSRX(motorId);
    motor.setInverted(inverted);
    this.rotsToRads = Units.rotationsToRadians(1) * gearRatio;

    FaultReporter.getInstance()
        .registerHardware(subsystemName, String.format("Motor %d", motorId), this.motor);
  }

  @Override
  public void updateInputs(SimpleMotorControllerIOInputs inputs) {
    inputs.position = motor.getSelectedSensorPosition() * rotsToRads;
    inputs.velocity = (motor.getSelectedSensorVelocity() / 60) * rotsToRads;
    inputs.appliedVolts = motor.getMotorOutputVoltage();
    inputs.current = new double[] {motor.getStatorCurrent()};
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }
}
