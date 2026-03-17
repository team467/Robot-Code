package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.TOLERANCE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.RobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public boolean controllerEnabled = true;
  private double targetRadPerSec = 0.0;
  private double rampedTarget = 0.0;

  // Feedforward: handles steady-state voltage (V = KS + KV * velocity)
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV);
  // PID: handles error correction on top of feedforward
  // P only — no I term (causes integral windup oscillation)
  private final PIDController pid = new PIDController(0.001, 0.1, 0.1, 0.02);

  // Slew rate limiter: ramps the target velocity gradually (rad/s per second)
  // This prevents current spikes that cause oscillation with a 20A limit
  private final SlewRateLimiter targetRamper = new SlewRateLimiter(800);

  // Fire control solver — handles shoot-on-the-move, drag compensation, LUT lookup
  private final ShotCalculator shotCalculator;
  private ShotCalculator.LaunchParameters lastLaunchParams =
      ShotCalculator.LaunchParameters.INVALID;

  public Shooter(ShooterIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new Config(
                Volts.per(Second).of(1),
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // Set up fire control
    ShotCalculator.Config calcConfig = new ShotCalculator.Config();
    calcConfig.launcherOffsetX = ShooterConstants.LAUNCHER_OFFSET_X;
    calcConfig.launcherOffsetY = ShooterConstants.LAUNCHER_OFFSET_Y;
    calcConfig.phaseDelayMs = ShooterConstants.PHASE_DELAY_MS;
    calcConfig.mechLatencyMs = ShooterConstants.MECH_LATENCY_MS;
    calcConfig.sotmDragCoeff = ShooterConstants.DRAG_COEFF;
    shotCalculator = new ShotCalculator(calcConfig);

    // Generate lookup table from projectile simulation
    ProjectileSimulator sim = new ProjectileSimulator(ShooterConstants.SIM_PARAMS);
    ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();
    for (ProjectileSimulator.LUTEntry entry : lut.entries()) {
      if (entry.reachable()) {
        shotCalculator.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
      }
    }
    Logger.recordOutput("Shooter/LUT/ReachableCount", lut.reachableCount());
    Logger.recordOutput("Shooter/LUT/MaxRangeM", lut.maxRangeM());
    Logger.recordOutput("Shooter/LUT/GenerationTimeMs", lut.generationTimeMs());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    RobotState.getInstance().shooterAtSpeed = isAtSetpoint() && targetRadPerSec > 0;
    if (controllerEnabled) {
      // Ramp toward the target to avoid current spikes
      rampedTarget = targetRamper.calculate(targetRadPerSec);

      double ff = feedforward.calculate(rampedTarget);
      double pidOutput = pid.calculate(inputs.shooterWheelVelocityRadPerSec, rampedTarget);
      double voltage = ff;

      // Clamp to valid voltage range
      voltage =
          Math.max(-ShooterConstants.MAX_VOLTAGE, Math.min(ShooterConstants.MAX_VOLTAGE, voltage));
      io.setVoltage(voltage);

      Logger.recordOutput("Shooter/FFVoltage", ff);
      Logger.recordOutput("Shooter/PIDVoltage", pidOutput);
      Logger.recordOutput("Shooter/CommandedVoltage", voltage);
      Logger.recordOutput("Shooter/RampedTargetRadPerSec", rampedTarget);
    }

    // Log fire control state
    Logger.recordOutput("Shooter/FireControl/Valid", lastLaunchParams.isValid());
    Logger.recordOutput("Shooter/FireControl/Confidence", lastLaunchParams.confidence());
    Logger.recordOutput("Shooter/FireControl/RPM", lastLaunchParams.rpm());
    Logger.recordOutput("Shooter/FireControl/SolvedDistanceM", lastLaunchParams.solvedDistanceM());

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/AtSetpoint", isAtSetpoint());
  }

  /**
   * Run the fire control solver for shoot-on-the-move. Call this each cycle from Orchestrator.
   * Returns the launch parameters (RPM, heading, confidence).
   */
  public ShotCalculator.LaunchParameters solve(
      Pose2d robotPose,
      ChassisSpeeds fieldVelocity,
      ChassisSpeeds robotVelocity,
      Translation2d hubCenter,
      Translation2d hubForward,
      double visionConfidence) {
    ShotCalculator.ShotInputs shotInputs =
        new ShotCalculator.ShotInputs(
            robotPose, fieldVelocity, robotVelocity, hubCenter, hubForward, visionConfidence);
    lastLaunchParams = shotCalculator.calculate(shotInputs);
    return lastLaunchParams;
  }

  public ShotCalculator.LaunchParameters getLastLaunchParams() {
    return lastLaunchParams;
  }

  /** Copilot RPM trim — bind to D-pad for in-match adjustments. */
  public void adjustRpmOffset(double delta) {
    shotCalculator.adjustOffset(delta);
  }

  public void resetRpmOffset() {
    shotCalculator.resetOffset();
  }

  /** Reset warm start after pose reset so solver doesn't use stale data. */
  public void resetSolverState() {
    shotCalculator.resetWarmStart();
  }

  public void runCharacterization(double voltage) {
    io.setVoltage(voltage);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> controllerEnabled = false, this)
        .andThen(run(() -> runCharacterization(0.0)).withTimeout(1.0))
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.runOnce(() -> controllerEnabled = false, this)
        .andThen(run(() -> runCharacterization(0.0)).withTimeout(1.0))
        .andThen(sysId.dynamic(direction));
  }

  public Command stop() {
    return Commands.runOnce(
            () -> {
              controllerEnabled = false;
              targetRadPerSec = 0.0;
              rampedTarget = 0.0;
              targetRamper.reset(0.0);
              io.stop();
            },
            this)
        .withName("shooter_stop_please");
  }

  public Command setVoltage(double volts) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerEnabled = false, this),
        Commands.run(() -> io.setVoltage(volts), this));
  }

  public Command setTargetVelocityRPM(double rpm) {
    return Commands.run(
            () -> {
              targetRadPerSec = ((rpm / 2 * Math.PI) * 60.0);
              controllerEnabled = true;
            },
            this)
        .withName("setTargetVelocityRPM");
  }

  public Command setTargetVelocityRadians(double radPerSec) {
    return Commands.run(
            () -> {
              targetRadPerSec = radPerSec;
              controllerEnabled = true;
            },
            this)
        .withName("setTargetVelocityRadians");
  }

  public Command setTargetVelocityRadians(DoubleSupplier radPerSec) {
    return Commands.run(
            () -> {
              targetRadPerSec = radPerSec.getAsDouble();
              controllerEnabled = true;
            },
            this)
        .withName("setTargetVelocityRadians");
  }

  /**
   * Continuously re-asserts shooter target while scheduled.
   *
   * <p>Useful for toggle/manual control flows where another command may temporarily disable the
   * shooter controller.
   */
  public Command setTargetVelocityRadiansRepeatedly(double radPerSec) {
    return Commands.repeatingSequence(
            Commands.runOnce(
                () -> {
                  targetRadPerSec = radPerSec;
                  controllerEnabled = true;
                },
                this),
            Commands.waitSeconds(0.02))
        .withName("setTargetVelocityRadiansRepeatedly");
  }

  public boolean isAtSetpoint() {
    return Math.abs(inputs.shooterWheelVelocityRadPerSec - (targetRadPerSec)) < TOLERANCE;
  }

  /** Time-of-flight from the LUT at the given distance (seconds). */
  public double getAirTimeSeconds(DoubleSupplier distance) {
    return shotCalculator.getTimeOfFlight(distance.getAsDouble());
  }

  /** RPM setpoint for the given distance, converted to rad/s. */
  public double calculateSetpoint(DoubleSupplier distance) {
    double rpm = shotCalculator.getBaseRPM(distance.getAsDouble());
    return rpm * (2.0 * Math.PI) / 60.0;
  }

  /** Get the launch velocity (m/s) for a given RPM — useful for sim visualization. */
  public double getLaunchVelocity(double rpm) {
    return ShooterConstants.SLIP_FACTOR * rpm * Math.PI * ShooterConstants.WHEEL_DIAMETER_M / 60.0;
  }

  public double getSetpoint() {
    return targetRadPerSec;
  }
}
