package frc.robot.subsystems.elevator;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  private TrapezoidProfile profile;
  private TrapezoidProfile.State lastProfiledReference;
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private final LinearSystem<N2, N1, N2> elevatorPlant;
  private final KalmanFilter<N2, N1, N1> observer;
  private final LinearQuadraticRegulator<N2, N1, N1> controller;
  private final LinearSystemLoop<N2, N1, N1> loop;

  private boolean feedbackMode = true;

  @AutoLogOutput private boolean isCalibrated = false;
  @AutoLogOutput private boolean holdLock = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();
    io.updateInputs(inputs);

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY.get(), ElevatorConstants.MAX_ACCELERATION.get()));
    /*
    States: [position, velocity], in meters and meters per second.
    Inputs (what we can "put in"): [voltage], in volts.
    Outputs (what we can measure): [position], in meters.
     */
    elevatorPlant =
        LinearSystemId.createElevatorSystem(
            DCMotor.getNEO(1),
            ElevatorConstants.CARRIAGE_MASS_KG,
            ElevatorConstants.DRUM_RADIUS_METERS,
            ElevatorConstants.ELEVATOR_GEARING);

    observer =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
            VecBuilder.fill(
                Units.inchesToMeters(2),
                Units.inchesToMeters(
                    40)), // How accurate we think our model is, in meters and meters/second.
            VecBuilder.fill(
                0.001), // How accurate we think our encoder position data is. In this case we very
            // highly trust our encoder position reading.
            0.02);

    controller =
        new LinearQuadraticRegulator<>(
            (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
            VecBuilder.fill(
                Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)), // qelms. Position
            // and velocity error tolerances, in meters and meters per second. Decrease this to more
            // heavily penalize state excursion, or make the controller behave more aggressively. In
            // this example we weight position much more highly than velocity, but this can be
            // tuned to balance the two.
            VecBuilder.fill(
                12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); // Nominal time between loops.

    loop =
        new LinearSystemLoop<>(
            (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0), controller, observer, 12.0, 0.02);

    // Reset last loop and reference to current state to make sure it's in a known state
    loop.reset(VecBuilder.fill(inputs.positionMeters, inputs.velocityMetersPerSec));
    lastProfiledReference =
        new TrapezoidProfile.State(inputs.positionMeters, inputs.velocityMetersPerSec);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Elevator", inputs);

    if (Constants.tuningMode) {
      if (ElevatorConstants.MAX_VELOCITY.hasChanged(hashCode())
          || ElevatorConstants.MAX_ACCELERATION.hasChanged(hashCode())) {
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    ElevatorConstants.MAX_VELOCITY.get(),
                    ElevatorConstants.MAX_ACCELERATION.get()));
      }
    }

    if (inputs.limitSwitchPressed) {
      io.resetPosition();
      if (!isCalibrated) {
        loop.reset(VecBuilder.fill(ElevatorConstants.STOW, inputs.velocityMetersPerSec));
        lastProfiledReference = new State(ElevatorConstants.STOW, inputs.velocityMetersPerSec);
        isCalibrated = true;
      }
    }

    Logger.recordOutput("Elevator/PIDEnabled", feedbackMode);
    if (feedbackMode) {
      lastProfiledReference = profile.calculate(0.02, lastProfiledReference, goal);
      loop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);
      loop.correct(VecBuilder.fill(inputs.positionMeters));
      loop.predict(0.02);
      io.setVoltage(loop.getU(0));

      Logger.recordOutput("Elevator/Goal/Position", goal.position);
      Logger.recordOutput("Elevator/Goal/Velocity", goal.velocity);
      Logger.recordOutput("Elevator/Setpoint/Position", lastProfiledReference.position);
      Logger.recordOutput("Elevator/Setpoint/Velocity", lastProfiledReference.velocity);
      Logger.recordOutput("Elevator/Measurement/Position", inputs.positionMeters);
    } else {
      lastProfiledReference =
          new TrapezoidProfile.State(inputs.positionMeters, inputs.velocityMetersPerSec);
    }
  }

  public Command toSetpoint(double setpointAngle) {
    return Commands.run(
        () -> {
          goal = new State(setpointAngle, 0);
          feedbackMode = true;
        },
        this);
  }

  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Elevator/DesiredVolts", percent * 12);
          io.setPercent(percent);
          feedbackMode = false;
        },
        this);
  }

  public Command hold() {
    return Commands.sequence(
        Commands.sequence(
                Commands.runOnce(() -> io.setVoltage(0.05 * 12), this), Commands.waitSeconds(0.5))
            .onlyIf(() -> inputs.velocityMetersPerSec < 0),
        Commands.run(
                () -> {
                  if (!holdLock) {
                    holdLock = true;
                    if (inputs.velocityMetersPerSec < 0) {
                      goal = new State(inputs.positionMeters, 0);
                    } else {
                      goal = new State(inputs.positionMeters, 0);
                    }
                    feedbackMode = true;
                  }
                },
                this)
            .finallyDo(() -> holdLock = false));
  }

  public double getPosition() {
    return inputs.positionMeters;
  }

  public boolean atSetpoint() {
    return loop.getError(0) < 0.05 && goal.equals(lastProfiledReference);
  }

  public boolean limitSwitchPressed() {
    return inputs.limitSwitchPressed;
  }
}
