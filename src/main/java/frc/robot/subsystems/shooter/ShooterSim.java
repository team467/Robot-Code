package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.GEAR_RATIO;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.RoboRioSim;

public class ShooterSim {

  private final DCMotor motors = DCMotor.getNEO(4);
  private final FlywheelSim flywheelSim = new FlywheelSim(
      Models.flywheelFromPhysicalConstants(motors, 0.02, GEAR_RATIO), motors);
  private SparkMaxSim sim;

  public void init(SparkMax leader) {
    sim = new SparkMaxSim(leader, DCMotor.getNEO(1));
  }

  public void getPeriodic() {
    double voltage = sim.getBusVoltage() * sim.getAppliedOutput();

    flywheelSim.setInputVoltage(voltage);

    flywheelSim.update(0.02);

    sim.iterate(RadiansPerSecond.of(flywheelSim.getAngularVelocity()).in(RPM),
        RoboRioSim.getVInVoltage(), 0.02);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
        Math.max(0.0, flywheelSim.getCurrentDraw())));
  }
}
