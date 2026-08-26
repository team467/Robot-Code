package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ClimberIOSim implements ClimberIO {
  private double positionDegrees = STARTING_DEGREES;
  private double targetRotation = STARTING_DEGREES;
  private double appliedVolts = 0.0;
  private boolean isCalibrated = true;
  private final PIDController pidController = new PIDController(CLIMBER_KP, CLIMBER_KI, CLIMBER_KD);

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    double velocity = (appliedVolts / 12.0) * 180.0; // deg/sec
    positionDegrees += velocity * 0.02;

    inputs.positionDegrees = positionDegrees;
    inputs.velocityDegreesPerSec = velocity;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(appliedVolts) * 2.0;
    inputs.targetRotation = targetRotation;
    inputs.atTargetRotation = Math.abs(targetRotation - positionDegrees) < TOLERANCE;
    inputs.limitSwitch = positionDegrees <= CALIBRATION_POSITION_DEGREES + 1.0;
    inputs.isCalibrated = isCalibrated;
  }

  @Override
  public void setPercent(double percent) {
    this.appliedVolts = MathUtil.clamp(percent, -1.0, 1.0) * 12.0;
  }

  @Override
  public void setRotation(double degrees) {
    this.targetRotation = degrees;
  }

  @Override
  public void goToRotation() {
    double pidVolts = pidController.calculate(positionDegrees, targetRotation);
    this.appliedVolts = MathUtil.clamp(pidVolts, -12.0, 12.0);
  }
}
