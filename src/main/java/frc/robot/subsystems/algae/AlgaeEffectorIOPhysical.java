package frc.robot.subsystems.algae;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeEffectorIOPhysical implements AlgaeEffectorIO {

  private final SparkMax pivotMotor;
  private final SparkMax removalMotor;
  // TODO: Move to schematic
  private static final int PIVOT_ID = 1;
  private static final int REMOVAL = 1;

  public AlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    removalMotor = new SparkMax(REMOVAL, MotorType.kBrushless);
  }

  public void setRemovalVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void setPivotVolts(double volts) {
    removalMotor.setVoltage(volts);
  }
}
