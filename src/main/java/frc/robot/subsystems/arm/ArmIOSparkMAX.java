package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ArmIOSparkMAX implements ArmIO {
  private final CANSparkMax leader;
  private final CANSparkMax follower;

  public ArmIOSparkMAX() {
    leader = new CANSparkMax(0, MotorType.kBrushless);
    follower = new CANSparkMax(1, MotorType.kBrushless);
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
    inputs.positionRads = Rotation2d.fromRadians(leader.getEncoder().getPosition());
    inputs.velocityRadsPerSec = leader.getEncoder().getVelocity();
    inputs.armAppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.armCurrentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }
}
