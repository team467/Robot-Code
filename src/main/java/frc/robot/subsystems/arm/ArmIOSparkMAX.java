package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.util.Units;

public class ArmIOSparkMAX implements ArmIO {
  private final CANSparkMax leader;
  private final CANSparkMax follower;
  private final SparkLimitSwitch leaderLimitSwitch;

  public ArmIOSparkMAX() {
    leader = new CANSparkMax(10, MotorType.kBrushless);
    follower = new CANSparkMax(11, MotorType.kBrushless);
    leaderLimitSwitch = leader.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    leader.getEncoder().setPositionConversionFactor(Units.rotationsToRadians(1) / 199.73);
    leader
        .getEncoder()
        .setVelocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1) / 199.73);
    leader.enableVoltageCompensation(12);
    follower.enableVoltageCompensation(12);
    follower.follow(leader);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRads = leader.getEncoder().getPosition();
    inputs.velocityRadsPerSec = leader.getEncoder().getVelocity();
    inputs.armAppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.armCurrentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.limitSwitchPressed = leaderLimitSwitch.isPressed();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
    follower.setVoltage(volts);
  }

  @Override
  public void resetPosition() {
    leader.getEncoder().setPosition(0.0);
    follower.getEncoder().setPosition(0.0);
  }
}
