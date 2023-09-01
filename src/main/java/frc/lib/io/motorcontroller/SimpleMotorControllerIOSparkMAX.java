package frc.lib.io.motorcontroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.autocheck.FaultReporter;

/** A Motor Controller IO class that interacts with a Spark MAX. */
public class SimpleMotorControllerIOSparkMAX implements SimpleMotorControllerIO {

  private final CANSparkMax motor;

  /**
   * Create a new SimpleMotorControllerIOSparkMAX
   *
   * @param motorId The ID of the motor
   * @param inverted Whether the motor is inverted
   * @param gearRatio The gear ratio in inputs/outputs
   * @param subsystemName The name of the subsystem using the motor controller, for use with the
   *     self-check tool
   */
  public SimpleMotorControllerIOSparkMAX(
      int motorId, boolean inverted, double gearRatio, String subsystemName) {
    this.motor = new CANSparkMax(motorId, MotorType.kBrushless);
    this.motor.setInverted(inverted);
    this.motor.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) * gearRatio);
    this.motor
        .getEncoder()
        .setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1));

    FaultReporter.getInstance()
        .registerHardware(subsystemName, String.format("Motor %d", motorId), this.motor);
  }

  @Override
  public void updateInputs(SimpleMotorControllerIOInputs inputs) {
    inputs.position = motor.getEncoder().getPosition();
    inputs.velocity = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.current = new double[] {motor.getOutputCurrent()};
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
