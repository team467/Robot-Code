package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.util.Units;
import frc.robot.Schematic;

public class ArmIOSparkMAX implements ArmIO {
  private final CANSparkMax leader;
  private final CANSparkMax follower;
  private final SparkLimitSwitch leaderLimitSwitch;

  public ArmIOSparkMAX() {
    leader = new CANSparkMax(Schematic.ARM_ID_LEADER, MotorType.kBrushless);
    follower = new CANSparkMax(Schematic.ARM_ID_FOLLOWER, MotorType.kBrushless);
    leaderLimitSwitch = leader.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    leader
        .getEncoder()
        .setPositionConversionFactor(
            Units.rotationsToRadians(1) * ArmConstants.GEAR_RATIO.getRotationsPerInput());
    leader
        .getEncoder()
        .setVelocityConversionFactor(
            Units.rotationsPerMinuteToRadiansPerSecond(1)
                * ArmConstants.GEAR_RATIO.getRotationsPerInput());
    leader.enableVoltageCompensation(12);
    follower.enableVoltageCompensation(12);

    leader.setIdleMode(CANSparkBase.IdleMode.kBrake);
    follower.setIdleMode(CANSparkBase.IdleMode.kBrake);

    follower.follow(leader, true);
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
    leader.getEncoder().setPosition(ArmConstants.OFFSET.getRadians());
    follower.getEncoder().setPosition(ArmConstants.OFFSET.getRadians());
  }
}
