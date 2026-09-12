package frc.robot.subsystems.intake.extend;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class IntakeExtendIOSim implements IntakeExtendIO {
  private double currentPos = COLLAPSE_POS;
  private double setpoint = COLLAPSE_POS;
  private double appliedVolts = 0.0;
  private double extendPercent = 0.0;
  private boolean usingPID = false;
  private final PIDController pidController = new PIDController(PID_P, PID_I, PID_D);

  public IntakeExtendIOSim() {
    pidController.setTolerance(POSITION_TOLERANCE);
  }

  @Override
  public void updateInputs(IntakeExtendIOInputs inputs) {
    if (edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
      usingPID = false;
      appliedVolts = 0.0;
      extendPercent = 0.0;
      pidController.reset();
    } else if (usingPID) {
      double pidOutput = pidController.calculate(currentPos, setpoint);
      appliedVolts = MathUtil.clamp(pidOutput * 12.0, -12.0, 12.0);
      extendPercent = appliedVolts / 12.0;
    }

    boolean disabled = edu.wpi.first.wpilibj.DriverStation.isDisabled();

    // Update position based on applied output (max speed ~ 3.5 pos units per second)
    double velocity = disabled ? 0.0 : ((appliedVolts / 12.0) * 3.5);
    currentPos += velocity * 0.02;

    // Physical stops
    currentPos = MathUtil.clamp(currentPos, EXTEND_POS - 0.05, COLLAPSE_POS + 0.05);

    inputs.getExtendPos = currentPos;
    inputs.extendVelocity = velocity;
    inputs.extendVolts = disabled ? 0.0 : appliedVolts;
    inputs.extendPercentOutput = disabled ? 0.0 : extendPercent;
    inputs.extendAmps = disabled ? 0.0 : (Math.abs(appliedVolts) * 1.2);
    inputs.isCollapsed = Math.abs(currentPos - COLLAPSE_POS) < 0.05;
    inputs.hasSetpoint = usingPID && !disabled;
    inputs.setpointValue = setpoint;
    inputs.atSetpoint =
        usingPID && !disabled && (Math.abs(currentPos - setpoint) <= POSITION_TOLERANCE);
    inputs.stowed = inputs.isCollapsed;
  }

  @Override
  public void setPercentExtend(double extendPercent) {
    this.usingPID = false;
    this.extendPercent = MathUtil.clamp(extendPercent, -1.0, 1.0);
    this.appliedVolts = this.extendPercent * 12.0;
  }

  @Override
  public void setVoltageExtend(double extendVolts) {
    this.usingPID = false;
    this.appliedVolts = MathUtil.clamp(extendVolts, -12.0, 12.0);
    this.extendPercent = this.appliedVolts / 12.0;
  }

  @Override
  public void stop() {
    this.usingPID = false;
    this.appliedVolts = 0.0;
    this.extendPercent = 0.0;
  }

  @Override
  public void goToPos(double pos) {
    this.setpoint = pos;
    this.usingPID = true;
    pidController.setSetpoint(pos);
  }

  @Override
  public void setPIDEnabled(boolean enabled) {
    this.usingPID = enabled;
  }

  @Override
  public boolean getPIDEnabled() {
    return usingPID;
  }

  @Override
  public void resetExtendEncoder(double position) {
    this.currentPos = position;
  }

  @Override
  public boolean isCollapsed() {
    return Math.abs(currentPos - COLLAPSE_POS) < 0.05;
  }

  @Override
  public void extendToPosition(double position) {
    goToPos(position);
  }
}
