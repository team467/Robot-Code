package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs;
  private ArmFeedforward feedforward =
      new ArmFeedforward(ArmConstants.KS.get(), ArmConstants.KG.get(), ArmConstants.KV.get());
  private ProfiledPIDController feedback =
      new ProfiledPIDController(
          ArmConstants.KP.get(),
          0,
          ArmConstants.KD.get(),
          new TrapezoidProfile.Constraints(
              ArmConstants.MAX_VELOCITY.get(), ArmConstants.MAX_ACCELERATION.get()));
  private boolean feedbackMode = true;
  @AutoLogOutput private boolean holdLock = false;

  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
    io.updateInputs(inputs);

    feedback.disableContinuousInput();
    feedback.reset(inputs.positionRads);
    feedback.setGoal(inputs.positionRads); // don't go crazy and kill someone
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    if (DriverStation.isDisabled()) {
      feedback.reset(inputs.positionRads, inputs.velocityRadsPerSec);
    }

    if (Constants.tuningMode) {
      // Update controllers if tunable numbers have changed
      if (ArmConstants.KS.hasChanged(hashCode())
          || ArmConstants.KG.hasChanged(hashCode())
          || ArmConstants.KV.hasChanged(hashCode())) {
        feedforward =
            new ArmFeedforward(ArmConstants.KS.get(), ArmConstants.KG.get(), ArmConstants.KV.get());
      }
      if (ArmConstants.KP.hasChanged(hashCode())
          || ArmConstants.KD.hasChanged(hashCode())
          || ArmConstants.MAX_VELOCITY.hasChanged(hashCode())
          || ArmConstants.MAX_ACCELERATION.hasChanged(hashCode())) {
        feedback.setPID(ArmConstants.KP.get(), 0, ArmConstants.KD.get());
        feedback.setConstraints(
            new TrapezoidProfile.Constraints(
                ArmConstants.MAX_VELOCITY.get(), ArmConstants.MAX_ACCELERATION.get()));
      }
    }

    if (inputs.limitSwitchPressed) {
      io.resetPosition();
      feedback.reset(ArmConstants.OFFSET.getRadians());
    }

    Logger.recordOutput("Arm/PIDEnabled", feedbackMode);
    if (feedbackMode) {
      logTrapzezoidTimes();
      // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html#goal-vs-setpoint
      io.setVoltage(
          feedback.calculate(inputs.positionRads)
              + feedforward.calculate(
                  feedback.getSetpoint().position, feedback.getSetpoint().velocity));
      // Logger.recordOutput("Arm/DesiredPosition", feedback.getGoal().position);
      // Logger.recordOutput("Arm/NextPosition", feedback.getSetpoint().position);
      // Logger.recordOutput("Arm/NextVelocity", feedback.getSetpoint().velocity);

      Logger.recordOutput("Arm/Goal/Position", feedback.getGoal().position);
      Logger.recordOutput("Arm/Goal/Velocity", feedback.getGoal().velocity);
      Logger.recordOutput("Arm/Setpoint/Position", feedback.getSetpoint().position);
      Logger.recordOutput("Arm/Setpoint/Velocity", feedback.getSetpoint().velocity);
      Logger.recordOutput("Arm/Measurement/Position", inputs.positionRads);
    } else {
      feedback.reset(new TrapezoidProfile.State(inputs.positionRads, inputs.velocityRadsPerSec));
    }
  }

  /**
   * Sets the arm to a specified setpoint angle.
   *
   * @param setpointAngle The desired angle to set the arm to.
   * @return The command to set the arm to the specified setpoint angle.
   */
  public Command toSetpoint(Rotation2d setpointAngle) {
    return Commands.run(
        () -> {
          feedback.setGoal(setpointAngle.getRadians());
          feedbackMode = true;
        },
        this);
  }

  /**
   * Runs the command with a specified percentage of voltage.
   *
   * @param percent The percentage of 12 volts to run [0,1].
   * @return The command to run with the specified voltage percentage.
   */
  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          Logger.recordOutput("Arm/DesiredVolts", percent * 12);
          io.setVoltage(percent * 12);
          feedbackMode = false;
        },
        this);
  }

  public Command hold() {
    return Commands.run(
            () -> {
              if (!holdLock) {
                holdLock = true;
                feedback.setGoal(inputs.positionRads);
                feedbackMode = true;
              }
            },
            this)
        .finallyDo(() -> holdLock = false);
  }

  private void logTrapzezoidTimes() {
    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity

    TrapezoidProfile.State m_current = feedback.getSetpoint();
    TrapezoidProfile.State goal = feedback.getGoal();
    TrapezoidProfile.Constraints m_constraints = feedback.getConstraints();

    double cutoffBegin = m_current.velocity / m_constraints.maxAcceleration;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

    double cutoffEnd = goal.velocity / m_constraints.maxAcceleration;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    double fullTrapezoidDist =
        cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd;
    double accelerationTime = m_constraints.maxVelocity / m_constraints.maxAcceleration;

    double fullSpeedDist =
        fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
      accelerationTime = Math.sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
      fullSpeedDist = 0;
    }

    Logger.recordOutput("Arm/m_endAccel", accelerationTime - cutoffBegin);
    Logger.recordOutput(
        "Arm/m_endFullSpeed",
        (accelerationTime - cutoffBegin) + fullSpeedDist / m_constraints.maxVelocity);
    Logger.recordOutput(
        "Arm/m_endDeccel",
        ((accelerationTime - cutoffBegin) + fullSpeedDist / m_constraints.maxVelocity)
            + accelerationTime
            - cutoffEnd);
    //    m_endAccel = accelerationTime - cutoffBegin;
    //    m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
    //    m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
  }
}
