package frc.robot.subsystems.intake.extend;

import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POS;
import static frc.robot.subsystems.intake.IntakeConstants.HOME_VOLTAGE;
import static frc.robot.subsystems.intake.IntakeConstants.STALL_VELOCITY;

import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public class IntakeExtend implements Mechanism {

  private final IntakeExtendIO io;
  private final IntakeExtendIOInputsAutoLogged inputs = new IntakeExtendIOInputsAutoLogged();

  private boolean hasPose = false;

  public IntakeExtend(IntakeExtendIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/stallingExtend",
        Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts < -0.01);
    Logger.recordOutput("Intake/stallingCollapse",
        Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts > 0.01);
    if (inputs.getExtendPos > EXTEND_POS / 2) {
      RobotState.getInstance().intakePosition = IntakePosition.STOWED;
    }
    if (inputs.getExtendPos <= EXTEND_POS / 2) {
      RobotState.getInstance().intakePosition = IntakePosition.DEPLOYED;
    }
    if (inputs.isCollapsed) {
      if (!hasPose) {
        Logger.recordOutput("Intake/IntakeExtend/HomeCompleted", true);
      }
      io.resetExtendEncoder(0.0);
      hasPose = true;
    }
    inputs.hasPose = hasPose;
    Logger.recordOutput("Intake/IntakeExtend/HasPose", hasPose);
  }

  public void stopExtend() {
    io.setVoltageExtend(0);
  }

  public Command resetExtendPosition() {
    return Commands.runOnce(() -> io.resetExtendEncoder(0), this);
  }

  public Command runIntakeExtendVolts(double volts) {
    return Commands.run(
        () -> {
          io.setPIDEnabled(false);
          io.setVoltageExtend(volts);
        },
        this);
  }

  public Command stopExtendingCommand() {
    return Commands.run(this::stopExtend, this);
  }

  public Command homeExtend() {
    return Commands.run(
            () -> {
              io.setPIDEnabled(false);
              io.setVoltageExtend(HOME_VOLTAGE);
            },
            this)
        .beforeStarting(() -> Logger.recordOutput("Intake/IntakeExtend/HomeStarted", true))
        .until(() -> inputs.isCollapsed)
        .finallyDo(this::stopExtend)
        .withName("homeExtend");
  }

  public Command extendToAngle(double angle) {
    return new ConditionalCommand(homeExtend(), Commands.none(), () -> !hasPose)
        .andThen(
            Commands.run(
                    () -> {
                      io.setPIDEnabled(true);
                      io.goToPos(angle);
                    },
                    this)
                .until(() -> inputs.atSetpoint)
                .finallyDo(this::stopExtend)
                .withName("extendToAngle"));
  }
}
