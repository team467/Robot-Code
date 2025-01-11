package frc.robot.subsystems.algae;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeEffectorIOPhysical implements AlgaeEffectorIO {


  private final SparkMax pivotMotor;
  private final SparkMax intakeMotor;
  // TODO: Move to schematic
  private static final int PIVOT_ID = 1;
  private static final int INTAKE = 1;

  public AlgaeEffectorIOPhysical() {
    pivotMotor = new SparkMax(PIVOT_ID, MotorType.kBrushless);
    intakeMotor = new SparkMax(INTAKE, MotorType.kBrushless);
  }

  public void setIntakeVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void setPivotVolts(double volts) {
    intakeMotor.setVoltage(volts);
  }
}
