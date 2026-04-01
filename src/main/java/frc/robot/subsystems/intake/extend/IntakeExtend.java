package frc.robot.subsystems.intake.extend;

import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_POS;
import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_VOLTS;
import static frc.robot.subsystems.intake.IntakeConstants.POSITION_TOLERANCE;
import static frc.robot.subsystems.intake.IntakeConstants.STALL_TIME;
import static frc.robot.subsystems.intake.IntakeConstants.STALL_VELOCITY;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeExtend extends SubsystemBase {
  private final IntakeExtendIO io;
  private final IntakeExtendIOInputsAutoLogged inputs = new IntakeExtendIOInputsAutoLogged();
  private final Timer stallExtendTimer = new Timer();
  private final Timer stallCollapseTimer = new Timer();

  private boolean stalledExtend = false;
  private boolean stalledCollapse = false;
  private boolean isStowed = false;
  private boolean isExtended = false;

  private double targetExtendPosition = 0;

  /**
   * @param limitSwitchDisabled A supplier to return whether the limit switch is currently disabled
   *     or not. If disabled, uses slipping to control intake.
   */
  private final BooleanSupplier limitSwitchDisabled;

  public void periodic() {
    inputs.stalledExtended = stalledExtend;
    inputs.stalledCollapsed = stalledCollapse;
    inputs.stowed = isStowed;
    inputs.stallExtendTimer = stallExtendTimer.get();
    inputs.stallCollapseTimer = stallCollapseTimer.get();

    // Non-slipping control calibration based on the limit switch state
    if (!limitSwitchDisabled.getAsBoolean() && isHopperCollapsed()) {
      io.resetExtendEncoder(0);
      isStowed = true;
      stalledCollapse = false;
    }

    // Slipping system
    if (limitSwitchDisabled.getAsBoolean()) {

      // If we are stalling in extend, start timing, otherwise stop timing
      if (!stalledExtend && isStallingExtend()) {
        stallExtendTimer.start();
      } else {
        stallExtendTimer.stop();
        stallExtendTimer.reset();
      }

      // If we are not stalled extended but the timer has exceeded the stall time, set to stalled
      if (!stalledExtend && stallExtendTimer.get() > STALL_TIME) {
        stalledExtend = true;
        stallExtendTimer.stop();
        stallExtendTimer.reset();
        io.resetExtendEncoder(EXTEND_POS);
        isExtended = true;
      }

      // If we are stalling in collapse, start timing, otherwise stop timing
      if (!stalledCollapse && isStallingCollapse()) {
        stallCollapseTimer.start();
      } else {
        stallCollapseTimer.stop();
        stallCollapseTimer.reset();
      }

      // If we are not stalled collapsed but the timer has exceeded the stall time, set to stalled
      if (!stalledCollapse && stallCollapseTimer.get() > STALL_TIME) {
        stalledCollapse = true;
        stallCollapseTimer.stop();
        stallCollapseTimer.reset();
        io.resetExtendEncoder(0);
        isStowed = true;
      }

      // If we are moving, we are not stalled
      if (isMoving()) {
        stalledExtend = false;
        stalledCollapse = false;
        isExtended = false;
        isStowed = false;
      }
    }
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/IntakeExtend/StalledExtend", stalledExtend);
    Logger.recordOutput("Intake/IntakeExtended/StalledCollapse", stalledCollapse);
    if (inputs.getExtendPos > EXTEND_POS / 2) {
      RobotState.getInstance().intakePosition = IntakePosition.STOWED;
    }
    if (inputs.getExtendPos <= EXTEND_POS / 2) {
      RobotState.getInstance().intakePosition = IntakePosition.DEPLOYED;
    }
  }

  public IntakeExtend(IntakeExtendIO io, BooleanSupplier limitSwitchDisabled) {
    this.io = io;
    this.limitSwitchDisabled = limitSwitchDisabled;
  }

  public boolean isHopperCollapsed() {
    return io.isCollapsed();
  }

  private boolean isStallingExtend() {
    // For our volts, we are not getting the right velocity
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts > EXTEND_VOLTS;
  }

  private boolean isStallingCollapse() {
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts < COLLAPSE_VOLTS;
  }

  private boolean isMoving() {
    return Math.abs(inputs.extendVelocity) > STALL_VELOCITY;
  }

  private void setPercentExtend(double extendPercent) {
    io.setPercentExtend(extendPercent);
  }

  public void setVoltageExtend(double extendVolts) {
    io.setVoltageExtend(extendVolts);
  }

  public void stopExtend() {
    io.setVoltageExtend(0);
  }

  public Command resetExtendPosition() {
    return Commands.runOnce(() -> io.resetExtendEncoder(0), this);
  }

  public Command moveToExtendedPosition() {
    return Commands.run(() -> io.extendToPosition(EXTEND_POS))
        .until(() -> Math.abs(inputs.getExtendPos - EXTEND_POS) <= POSITION_TOLERANCE)
        .withName("moveToExtendedPosition");
  }

  public Command moveToCollapsedPosition() {
    return Commands.run(() -> io.extendToPosition(COLLAPSE_POS))
        .until(() -> Math.abs(inputs.getExtendPos - COLLAPSE_POS) <= POSITION_TOLERANCE)
        .withName("moveToCollapsedPosition");
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

  public Command extendToAngle(double angle) {
    return Commands.run(
            () -> {
              io.setPIDEnabled(true);
              io.goToPos(angle);
            },
            this)
        .until(() -> inputs.atSetpoint)
        .finallyDo(this::stopExtend)
        .withName("extendToAngle");
  }

  public Command holdAngle(double angle) {
    return Commands.run(
            () -> {
              io.setPIDEnabled(true);
              io.goToPos(angle);
            },
            this)
        .finallyDo(this::stopExtend)
        .withName("holdAngle");
  }
}
