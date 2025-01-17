package frc.robot.subsystems.algae;

import static frc.robot.Schematic.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.RelativeEncoder.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeEffectorIOPhysical implements AlgaeEffectorIO {

  // motors
  private final SparkMax pivotMotor;
  private final SparkMax removalMotor;
  private final RelativeEncoder pivotMotorEncoder;
  private final RelativeEncoder removalMotorEncoder;
  private final SparkLimitSwitch pivotMotorExtendLimitSwitch;
  private final SparkLimitSwitch pivotMotorStoweLimitSwitch;

  public AlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(pivotId, MotorType.kBrushless);
    removalMotor = new SparkMax(removalId, MotorType.kBrushless);

    pivotMotorEncoder = pivotMotor.getEncoder();
    removalMotorEncoder = removalMotor.getEncoder();

    pivotMotorExtendLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotMotorStoweLimitSwitch = pivotMotor.getReverseLimitSwitch();
  }

  public void setRemovalVolts(double volts) {
    removalMotor.setVoltage(volts);
  }

  public void setPivotVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.removalVolts = removalMotor.getBusVoltage() * removalMotor.getAppliedOutput();
    inputs.pivotVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.pivotVelocity = pivotMotorEncoder.getVelocity();
    inputs.removalVelocity = removalMotorEncoder.getVelocity();
    inputs.removalAmps = removalMotor.getOutputCurrent();
    inputs.pivotAmps = pivotMotor.getOutputCurrent();
    inputs.pivotPosition = pivotMotorEncoder.getPosition();
  }
}
