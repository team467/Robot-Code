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

  // limit switches
  // private final SparkLimitSwitch pivotMotorExtendLimitSwitch;
  // private final SparkLimitSwitch pivotMotorStowLimitSwitch;

  // motor encoders
  private final RelativeEncoder pivotMotorEncoder;
  private final RelativeEncoder removalMotorEncoder;

  public AlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(pivotId, MotorType.kBrushless);
    removalMotor = new SparkMax(removalId, MotorType.kBrushless);

    // pivotMotorExtendLimitSwitch = pivotMotor.getForwardLimitSwitch();
    // pivotMotorStowLimitSwitch = pivotMotor.getReverseLimitSwitch();

    pivotMotorEncoder = pivotMotor.getEncoder();
    removalMotorEncoder = removalMotor.getEncoder();
  }

  public void setRemovalVolts(double volts) {
    removalMotor.setVoltage(volts);
  }

  public void setPivotVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.pivotVolts = pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput();
    inputs.removalVolts = removalMotor.getBusVoltage() * removalMotor.getAppliedOutput();
    inputs.pivotAmps = pivotMotor.getOutputCurrent();
    inputs.removalAmps = removalMotor.getOutputCurrent();
    inputs.pivotVelocity = pivotMotorEncoder.getVelocity();
    inputs.removalVelocity = removalMotorEncoder.getVelocity();
    inputs.pivotPosition = pivotMotorEncoder.getPosition();
  }
}
