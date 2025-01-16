package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private static final double LOOP_PERIOD_SECS = 0.0;
  private static final double CLIMBER_ROTS_TO_METERS = 0.0; // Conversion factor
  private static final double MAX_POSITION_METERS = 0.0; // Example limit

  private final DCMotorSim leaderMotorSim = new DCMotorSim(1, DCMotor.getNEO(1));
  private final DCMotorSim followerMotorSim = new DCMotorSim(1, DCMotor.getNEO(1));

  private double leaderAppliedVolts = 0.0;
  private double followerAppliedVolts = 0.0;
  private boolean ratchetLocked = false;
  private boolean limitSwitchPressed = false;
  private double positionMeters = 0.0;

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    leaderMotorSim.update(LOOP_PERIOD_SECS);
    followerMotorSim.update(LOOP_PERIOD_SECS);

    double velocityMetersPerSecond =
        leaderMotorSim.getAngularVelocityRadPerSec() * CLIMBER_ROTS_TO_METERS;
    positionMeters += velocityMetersPerSecond * LOOP_PERIOD_SECS;

    // Enforce limits
    if (positionMeters >= MAX_POSITION_METERS) {
      positionMeters = MAX_POSITION_METERS;
      limitSwitchPressed = true;
    } else if (positionMeters < 0.0) {
      positionMeters = 0.0;
      limitSwitchPressed = false;
    }

    inputs.motorPercentOutput = leaderAppliedVolts / 12.0; // Percent output relative to 12V
    inputs.motorVoltage = leaderAppliedVolts;
    inputs.ClimberLeaderPosition = positionMeters;
    inputs.ClimberFollowerPosition = positionMeters;
    inputs.currentAmpsLeader = Math.abs(leaderMotorSim.getCurrentDrawAmps());
    inputs.currentAmpsFollower = Math.abs(followerMotorSim.getCurrentDrawAmps());
    inputs.ratchetLocked = ratchetLocked;
    inputs.limitSwitchPressed = limitSwitchPressed;
  }

  @Override
  public void setMotorsOutputPercent(double percentOutput) {
    if (!ratchetLocked) {
      leaderAppliedVolts = MathUtil.clamp(percentOutput * 12.0, -12.0, 12.0);
      followerAppliedVolts = leaderAppliedVolts;

      leaderMotorSim.setInputVoltage(leaderAppliedVolts);
      followerMotorSim.setInputVoltage(followerAppliedVolts);
    } else {
      leaderAppliedVolts = 0.0;
      followerAppliedVolts = 0.0;

      leaderMotorSim.setInputVoltage(0.0);
      followerMotorSim.setInputVoltage(0.0);
    }
  }

  @Override
  public void setRatchetLocked(boolean locked) {
    ratchetLocked = locked;
  }

  @Override
  public boolean getLimitSwitch() {
    return limitSwitchPressed;
  }

  @Override
  public void resetPosition() {
    positionMeters = 0.0;
    // Reset motor simulations to zero velocity
    leaderMotorSim.setInputVoltage(0.0);
    followerMotorSim.setInputVoltage(0.0);
  }
}
