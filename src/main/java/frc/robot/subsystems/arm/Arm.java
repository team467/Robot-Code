package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private boolean enableFB = true;

  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

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

    if (enableFB) {
      // Run arm to desired goal
      // note that for profiled pids, goal is the last result (i.e. set arm to 30 deg), and setpoint
      // is what result to reach in this next loop (i.e. set the arm to 5 degrees) due to
      // constraints.
      // Read more at
      // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html#goal-vs-setpoint
      Logger.recordOutput("Arm/DesiredPosition", feedback.getGoal().position);
      io.setVoltage(
          feedback.calculate(inputs.positionRads.getRadians())
              + feedforward.calculate(
                  feedback.getSetpoint().position + ArmConstants.horizontalOffset.getRadians(),
                  feedback.getSetpoint().velocity));
      Logger.recordOutput("Arm/NextPosition", feedback.getSetpoint().position);
      Logger.recordOutput("Arm/NextVelocity", feedback.getSetpoint().velocity);
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
          enableFB = true;
        },
        this);
  }

  public Command runPercent(double percent) {
    return Commands.run(
        () -> {
          io.setVoltage(percent * 12);
          enableFB = false;
        },
        this);
  }

  /**
   * Sets the arm to the desired position to pick up a note
   *
   * @return The command to set the arm to the pick-up position
   */
  public Command pickup() {
    return toSetpoint(new Rotation2d(0.0));
  }
}
