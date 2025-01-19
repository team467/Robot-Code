package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class AlgaeEffectorIOSim implements AlgaeEffectorIO {
  
  // motors
  private final DCMotorSim pivotMotorSim;
  private final DCMotorSim removalMotorSim;

  private double pivotMotorSimVolts;
  private double removalMotorSimVolts;
  
  // constants needed to set up DC motor system, change as needed
  // random Jkg value, change if needed 
  private static final double JKgMetersSquared = 0.025;
  // random gearing value, change if needed 
  private static final double gearing = 4.71;
  public static final DCMotor algaeGearbox = DCMotor.getNeoVortex(1);
 
  // limit switches? 

  public AlgaeEffectorIOSim() {
    pivotMotorSim = 
        new DCMotorSim(
          LinearSystemId.createDCMotorSystem(algaeGearbox, JKgMetersSquared, gearing),
          algaeGearbox);
    
    removalMotorSim = 
        new DCMotorSim(
          LinearSystemId.createDCMotorSystem(algaeGearbox, JKgMetersSquared, gearing),
          algaeGearbox);
  }

  public void setRemovalVolts(double volts) {
    removalMotorSimVolts = volts;
  }

  public void setPivotVolts(double volts) {
    pivotMotorSimVolts = volts;
  }

  @Override
  public void updateInputs(AlgaeEffectorIOInputs inputs) {
    inputs.removalVolts = removalMotorSimVolts;
    inputs.pivotVolts = pivotMotorSimVolts;
    inputs.pivotVelocity = pivotMotorSim.getAngularVelocity().in(Units.RadiansPerSecond);
    inputs.removalVelocity = removalMotorSim.getAngularVelocity().in(Units.RadiansPerSecond);
    inputs.removalAmps = removalMotorSim.getCurrentDrawAmps();
    inputs.pivotAmps = pivotMotorSim.getCurrentDrawAmps();
  }
}
