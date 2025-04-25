package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.COTS.WHEELS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private final SwerveModuleSimulation moduleSimulation;
  // reference to the simulated drive motor
  private final SimulatedMotorController.GenericMotorController driveMotor;
  // reference to the simulated turn motor
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private PIDController turnController = new PIDController(turnSimP, 0, turnSimD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    // configures a generic motor controller for drive motor
    // set a current limit of 60 amps
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(driveMotorCurrentLimit));
    this.turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(turnMotorCurrentLimit));
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, driveMotorReduction),
            driveGearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnGearbox, 0.004, turnMotorReduction),
            turnGearbox);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
  // specified by ModuleIO interface
  @Override
  public void setDriveOutputVoltage(Voltage voltage) {
    this.driveMotor.requestVoltage(voltage);
  }

  @Override
  // specified by ModuleIO interface
  public void setSteerOutputVoltage(Voltage voltage) {
    this.turnMotor.requestVoltage(voltage);
  }

  @Override
  // specified by ModuleIO interface
  public Rotation2d getSteerFacing() {
    return this.moduleSimulation.getSteerAbsoluteFacing();
  }

  @Override
  // specified by ModuleIO interface
  public Angle getSteerRelativePosition() {
    return moduleSimulation
        .getSteerRelativeEncoderPosition()
        .divide(
            COTS.ofMark4(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getNEO(1),
                    WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                    2 // Gear ratio (l3 gear ratio)
                    )
                .STEER_GEAR_RATIO);
  }

  @Override
  // specified by ModuleIO interface
  public Angle getDriveWheelPositioned() {
    return moduleSimulation.getDriveWheelFinalPosition();
  }
}
