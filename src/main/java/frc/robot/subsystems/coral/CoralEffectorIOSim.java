package frc.robot.subsystems.coral;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralEffectorIOSim implements CoralEffectorIO {

  private final DCMotor neo = DCMotor.getNEO(1).withReduction(1);

  private final SparkMax motor;
  private final SparkMaxSim motorSim;

  private final Mechanism2d mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Coral Effector Root", 10, 0);
  // private final MechanismLigament2d coraleffectorMech2d =
  // mech2dRoot.append(new MechanismLigament2d("Elevator", motorSim.getPositionMeters(), 90));

  public CoralEffectorIOSim(int motorId) {

    motor = new SparkMax(motorId, MotorType.kBrushless);
    motorSim = new SparkMaxSim(motor, neo);

    motorSim.enable();
    SmartDashboard.putData("Coral Effector Sim", mech2d);
  }

  @Override
  public void updateInputs(CoralEffectorIOInputs inputs) {

    motorSim.iterate(motor.get(), RobotController.getBatteryVoltage(), 0.020);

    inputs.appliedVolts = motorSim.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.currentAmps = motorSim.getMotorCurrent();
    inputs.temperature = motor.getMotorTemperature();
    inputs.hopperSeesCoral = false;
    inputs.hasCoral = false;
  }

  public void setEffectorVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }
}
