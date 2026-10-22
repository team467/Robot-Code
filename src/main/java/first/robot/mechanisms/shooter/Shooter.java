package first.robot.mechanisms.shooter;

import static first.robot.mechanisms.shooter.ShooterConstants.*;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.shooter.ShooterIO.ShooterIOOutputMode;
import first.robot.mechanisms.shooter.ShooterIO.ShooterIOOutputs;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Alert;

public class Shooter implements Mechanism {
  private static final double SYSID_RAMP_RATE = 1.0;
  private static final double SYSID_STEP_VOLTAGE = 7.0;
  private static final double SYSID_TIMEOUT = 10.0;

  public static enum SysIdDirection {
    FORWARD,
    REVERSE
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputs outputs = new ShooterIOOutputs();

  private final LoggedNetworkNumber tunedKp = new LoggedNetworkNumber("/Tuning/Shooter/KP", KP);

  private double lastWheelVelocity = 0.0;
  private final LinearFilter accelFilter = LinearFilter.movingAverage(5);
  private double wheelAccel = 0.0;
  private int settledLoops = 0;
  private double settleReferenceTarget = 0.0;
  private boolean saturated = false;
  private final Alert saturatedAlert =
      new Alert(
          "shooter-saturated",
          "Shooter is out of voltage headroom (low battery?); shots may fall short",
          Alert.Level.MEDIUM);

  private boolean controllerEnabled = false;
  private AngularVelocity targetSpeed = RadiansPerSecond.of(0);
  private double rampedTarget = 0.0;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);

  private final SlewRateLimiter targetRamper = new SlewRateLimiter(TARGET_RAMP_RATE);

  private final LoggedNetworkNumber tunedKs = new LoggedNetworkNumber("/Tuning/Shooter/KS", KS);
  private final LoggedNetworkNumber tunedKv = new LoggedNetworkNumber("/Tuning/Shooter/KV", KV);
  private final LoggedNetworkNumber tunedKa = new LoggedNetworkNumber("/Tuning/Shooter/KA", KA);
  private final LoggedNetworkNumber tunedFFScalar =
      new LoggedNetworkNumber("/Tuning/Shooter/FeedForward_Scalar", FF_SCALAR);
  private final LoggedNetworkNumber tunedAConst =
      new LoggedNetworkNumber("/Tuning/Shooter/ACONST", ACONST);
  private final LoggedNetworkNumber tunedBConst =
      new LoggedNetworkNumber("/Tuning/Shooter/BCONST", BCONST);
  private final LoggedNetworkNumber tunedCConst =
      new LoggedNetworkNumber("/Tuning/Shooter/CCONST", CCONST);

  public Shooter(ShooterIO io) {
    this.io = io;
    Scheduler.getDefault().addPeriodic(this::updateInputs);
  }

  private void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    wheelAccel =
        accelFilter.calculate((inputs.shooterWheelVelocityRadPerSec - lastWheelVelocity) / 0.02);
    lastWheelVelocity = inputs.shooterWheelVelocityRadPerSec;
    double target = targetSpeed.in(RadiansPerSecond);
    if (Math.abs(wheelAccel) > SETTLED_ACCEL
        || Math.abs(target - settleReferenceTarget) > TOLERANCE) {
      settledLoops = 0;
      settleReferenceTarget = target;
    } else {
      settledLoops++;
    }
    Logger.recordOutput("Shooter/WheelAccelRadPerSec2", wheelAccel);

    double scalar = tunedFFScalar.get();
    feedforward.setKs(tunedKs.get() * scalar);
    feedforward.setKv(tunedKv.get() * scalar);
    feedforward.setKa(tunedKa.get() * scalar);
  }

  public void applyOutputs() {
    if (controllerEnabled) {
      rampedTarget = targetRamper.calculate(targetSpeed.in(RadiansPerSecond));

      double ff = feedforward.calculate(rampedTarget);
      double feedback = tunedKp.get() * (rampedTarget - inputs.shooterWheelVelocityRadPerSec);
      double voltage = Math.clamp(ff + feedback, -MAX_VOLTAGE, MAX_VOLTAGE);
      outputs.mode = ShooterIOOutputMode.VOLTAGE;
      outputs.targetVolts = voltage;

      saturated = Math.abs(voltage) >= inputs.busVoltage - SATURATION_MARGIN_VOLTS;

      Logger.recordOutput("Shooter/FFVoltage", ff);
      Logger.recordOutput("Shooter/FeedbackVoltage", feedback);
      Logger.recordOutput("Shooter/CommandedVoltage", voltage);
      Logger.recordOutput("Shooter/RampedTargetRadPerSec", rampedTarget);
    } else {
      saturated = false;
    }
    Logger.recordOutput("Shooter/Setpoint", targetSpeed.in(RadiansPerSecond));
    Logger.recordOutput("Shooter/Saturated", saturated);
    saturatedAlert.set(saturated && !isAtSetpoint());

    io.applyOutputs(outputs);
  }

  private void runTargetVelocity(AngularVelocity velocity) {
    targetSpeed = velocity;
    controllerEnabled = true;
  }

  private void runVoltage(double volts) {
    controllerEnabled = false;
    outputs.mode = ShooterIOOutputMode.VOLTAGE;
    outputs.targetVolts = volts;
  }

  private void stopOutput() {
    controllerEnabled = false;
    targetSpeed = RadiansPerSecond.of(0);
    rampedTarget = 0.0;
    targetRamper.reset(0.0);
    outputs.mode = ShooterIOOutputMode.STOP;
  }

  public void runCharacterization(double voltage) {
    runVoltage(voltage);
  }

  @Override
  public Command idle() {
    return run(coroutine -> {
          stopOutput();
          coroutine.park();
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Shooter.Idle");
  }

  public Command stop() {
    return run(coroutine -> stopOutput()).named("shooter_stop_please");
  }

  public Command setVoltage(double volts) {
    return run(coroutine -> {
          runVoltage(volts);
          coroutine.park();
        })
        .named("Shooter.SetVoltage");
  }

  public Command setTargetVelocity(Supplier<AngularVelocity> velocity) {
    return run(coroutine -> {
          while (true) {
            runTargetVelocity(velocity.get());
            coroutine.yield();
          }
        })
        .named("setTargetVelocity");
  }

  public Command setTargetVelocity(AngularVelocity velocity) {
    return run(coroutine -> {
          runTargetVelocity(velocity);
          coroutine.park();
        })
        .named("setTargetVelocity");
  }

  public Command setTargetVelocityRepeatedly(AngularVelocity velocity) {
    return setTargetVelocity(velocity);
  }

  public Command shootAtHub(Supplier<Distance> targetDistance) {
    Supplier<AngularVelocity> setpoint = calculateSetpoint(targetDistance);
    return run(coroutine -> {
          while (true) {
            runTargetVelocity(setpoint.get());
            coroutine.yield();
          }
        })
        .named("Shooter.ShootAtHub");
  }

  public Command sysIdQuasistatic(SysIdDirection direction) {
    SysIdRoutineLog.State state =
        direction == SysIdDirection.FORWARD
            ? SysIdRoutineLog.State.QUASISTATIC_FORWARD
            : SysIdRoutineLog.State.QUASISTATIC_REVERSE;
    double sign = direction == SysIdDirection.FORWARD ? 1.0 : -1.0;
    return sysIdTest(state, time -> sign * SYSID_RAMP_RATE * time).named("Shooter.SysIdQuasistatic");
  }

  public Command sysIdDynamic(SysIdDirection direction) {
    SysIdRoutineLog.State state =
        direction == SysIdDirection.FORWARD
            ? SysIdRoutineLog.State.DYNAMIC_FORWARD
            : SysIdRoutineLog.State.DYNAMIC_REVERSE;
    double sign = direction == SysIdDirection.FORWARD ? 1.0 : -1.0;
    return sysIdTest(state, time -> sign * SYSID_STEP_VOLTAGE).named("Shooter.SysIdDynamic");
  }

  private NeedsNameBuilderStage sysIdTest(
      SysIdRoutineLog.State state, DoubleUnaryOperator voltage) {
    return run(coroutine -> {
          runVoltage(0.0);
          coroutine.wait(Seconds.of(1.0));

          Timer timer = new Timer();
          timer.start();
          try {
            while (timer.get() < SYSID_TIMEOUT) {
              runVoltage(voltage.applyAsDouble(timer.get()));
              Logger.recordOutput("Shooter/SysIdState", state.toString());
              coroutine.yield();
            }
          } finally {
            Logger.recordOutput("Shooter/SysIdState", SysIdRoutineLog.State.NONE.toString());
            runVoltage(0.0);
          }
        });
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean isAtSetpoint() {
    return Math.abs(inputs.shooterWheelVelocityRadPerSec - targetSpeed.in(RadiansPerSecond))
            < TOLERANCE
        && targetSpeed.gt(RadiansPerSecond.of(0));
  }

  @AutoLogOutput(key = "Shooter/ReadyToShoot")
  public boolean isReadyToShoot() {
    double target = targetSpeed.in(RadiansPerSecond);
    if (target <= 0) {
      return false;
    }
    boolean settled = settledLoops * 0.02 >= SETTLED_TIME;
    boolean closeEnough = inputs.shooterWheelVelocityRadPerSec >= READY_MIN_FRACTION * target;
    return isAtSetpoint() || (settled && closeEnough);
  }

  public boolean isSaturated() {
    return saturated;
  }

  public Time getAirTimeSeconds(Distance distance) {
    return Seconds.of(0.0617 * distance.in(Meters) + 0.872);
  }

  public Supplier<AngularVelocity> calculateSetpoint(Supplier<Distance> distance) {
    return () -> {
      double meters = distance.get().in(Meters);
      double rpm =
          tunedAConst.get() * Math.pow(meters, 2) + tunedBConst.get() * meters + tunedCConst.get();
      return RPM.of(Math.min(rpm, MAX_SETPOINT_RPM));
    };
  }

  public AngularVelocity getSetpoint() {
    return targetSpeed;
  }
}
