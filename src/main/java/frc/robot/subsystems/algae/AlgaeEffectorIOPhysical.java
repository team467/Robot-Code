package frc.robot.subsystems.algae;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeEffectorIOPhysical implements AlgaeEffectorIO {

  private final SparkMax pivotMotor;
  private final SparkMax removalMotor;
  private final SparkLimitSwitch pivotMotorExtendLimitSwitch;
  private final SparkLimitSwitch pivotMotorStoweLimitSwitch;

  // TODO: Move to schematic
  private static final int PIVOT_ID = 1;
  private static final int REMOVAL_ID = 2;

  public AlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    removalMotor = new SparkMax(REMOVAL_ID, MotorType.kBrushless);

    pivotMotorExtendLimitSwitch = pivotMotor.getForwardLimitSwitch();
    pivotMotorStoweLimitSwitch = pivotMotor.getReverseLimitSwitch();
  }

  public void setRemovalVolts(double volts) {
    removalMotor.setVoltage(volts);
  }

  public void setPivotVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }
}
