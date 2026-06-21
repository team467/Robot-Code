package frc.robot.subsystems.intake.extend;

import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_POS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POS;
import static frc.robot.subsystems.intake.IntakeConstants.HOME_VOLTAGE;
import static frc.robot.subsystems.intake.IntakeConstants.POSITION_TOLERANCE;
import static frc.robot.subsystems.intake.IntakeConstants.STALL_VELOCITY;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeExtend extends SubsystemBase {
  private final IntakeExtendIO io;
  private final IntakeExtendIOInputsAutoLogged inputs = new IntakeExtendIOInputsAutoLogged();
  private final Timer stallExtendTimer = new Timer();
  private final Timer stallCollapseTimer = new Timer();

  private boolean stalledExtend = false;
  private boolean stalledCollapse = false;
  private boolean isStowed = false;
  private boolean hasPose = false;

  /** Updates logged inputs and keeps robot-wide intake state in sync with the extension encoder. */
  @Override
  public void periodic() {
    inputs.stalledExtended = stalledExtend;
    inputs.stalledCollapsed = stalledCollapse;
    inputs.stowed = isStowed;
    inputs.stallExtendTimer = stallExtendTimer.get();
    inputs.stallCollapseTimer = stallCollapseTimer.get();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/IntakeExtend/StalledExtend", stalledExtend);
    Logger.recordOutput("Intake/IntakeExtended/StalledCollapse", stalledCollapse);
    Logger.recordOutput("Intake/stallingExtend", isStallingExtend());
    Logger.recordOutput("Intake/stallingCollapse", isStallingCollapse());
    if (inputs.extendPosition > EXTEND_POS / 2) {
      RobotState.getInstance().intakePosition = IntakePosition.STOWED;
    }
    if (inputs.extendPosition <= EXTEND_POS / 2) {
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

  /** Creates an intake extension subsystem using the selected hardware abstraction. */
  public IntakeExtend(IntakeExtendIO io) {
    this.io = io;
  }

  /** Returns whether the extension is currently on the collapsed limit switch. */
  public boolean isHopperCollapsed() {
    return io.isCollapsed();
  }

  /** Supplies the current extension position for commands that need a live sensor reading. */
  public DoubleSupplier getAngle() {
    return () -> inputs.extendPosition;
  }

  /** Sets the motor idle behavior through the IO layer. */
  public void setIdleMode(boolean idleMode) {
    io.setIdleMode(idleMode);
  }

  /** Resets the extension encoder to a known pose. */
  public Command setPose(double pose) {
    return Commands.runOnce(() -> io.resetExtendEncoder(pose), this);
  }

  /** Returns true when the extension is being driven outward but the encoder is barely moving. */
  private boolean isStallingExtend() {
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts < -0.01;
  }

  /** Returns true when the extension is being collapsed but the encoder is barely moving. */
  private boolean isStallingCollapse() {
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts > 0.01;
  }

  /** Drives the extension motor with open-loop voltage. */
  public void setVoltageExtend(double extendVolts) {
    io.setVoltageExtend(extendVolts);
  }

  /** Stops the extension motor immediately. */
  public void stopExtend() {
    io.setVoltageExtend(0);
  }

  /** Creates a one-shot command that zeros the extension encoder. */
  public Command resetExtendPosition() {
    return Commands.runOnce(() -> io.resetExtendEncoder(0), this);
  }

  /** Moves the extension to the configured deployed position and finishes at tolerance. */
  public Command moveToExtendedPosition() {
    return Commands.run(() -> io.extendToPosition(EXTEND_POS))
        .until(() -> Math.abs(inputs.extendPosition - EXTEND_POS) <= POSITION_TOLERANCE)
        .withName("moveToExtendedPosition");
  }

  /** Moves the extension to the configured collapsed position and finishes at tolerance. */
  public Command moveToCollapsedPosition() {
    return Commands.run(() -> io.extendToPosition(COLLAPSE_POS))
        .until(() -> Math.abs(inputs.extendPosition - COLLAPSE_POS) <= POSITION_TOLERANCE)
        .withName("moveToCollapsedPosition");
  }

  /** Creates a manual voltage command for operator-controlled extension movement. */
  public Command runIntakeExtendVolts(double volts) {
    return Commands.run(
        () -> {
          io.setPIDEnabled(false);
          io.setVoltageExtend(volts);
        },
        this);
  }

  /** Creates a command that keeps the extension stopped while scheduled. */
  public Command stopExtendingCommand() {
    return Commands.run(this::stopExtend, this);
  }

  /** Homes the extension by driving until the collapsed limit switch is reached. */
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

  /** Homes first if needed, then moves the extension to the requested encoder angle. */
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
