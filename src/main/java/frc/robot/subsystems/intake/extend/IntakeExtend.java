package frc.robot.subsystems.intake.extend;

import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_POS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POS;
import static frc.robot.subsystems.intake.IntakeConstants.HOME_VOLTAGE;
import static frc.robot.subsystems.intake.IntakeConstants.POSITION_TOLERANCE;
import static frc.robot.subsystems.intake.IntakeConstants.STALL_VELOCITY;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public enum State {
    IDLE,
    HOMING,
    COLLAPSING,
    COLLAPSED,
    EXTENDING,
    EXTENDED,
    MANUAL
  }

  public State state = State.IDLE;
  private double targetPosition = COLLAPSE_POS;
  private double manualVoltage = 0.0;

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
    Logger.recordOutput("Intake/IntakeExtend/State", state.name());

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

    // State machine updates and motor control
    switch (state) {
      case IDLE -> {
        io.setPIDEnabled(false);
        io.setVoltageExtend(0.0);
      }
      case HOMING -> {
        if (inputs.isCollapsed) {
          io.resetExtendEncoder(0.0);
          hasPose = true;
          state = State.COLLAPSED;
        } else {
          io.setPIDEnabled(false);
          io.setVoltageExtend(HOME_VOLTAGE);
        }
      }
      case COLLAPSING -> {
        if (!hasPose) {
          state = State.HOMING;
        } else if (inputs.isCollapsed
            || Math.abs(inputs.extendPosition - targetPosition) <= POSITION_TOLERANCE) {
          state = State.COLLAPSED;
        } else {
          io.setPIDEnabled(true);
          io.goToPos(targetPosition);
        }
      }
      case COLLAPSED -> {
        io.setPIDEnabled(true);
        io.goToPos(COLLAPSE_POS);
      }
      case EXTENDING -> {
        if (!hasPose) {
          state = State.HOMING;
        } else if (Math.abs(inputs.extendPosition - targetPosition) <= POSITION_TOLERANCE) {
          if (targetPosition == EXTEND_POS) {
            state = State.EXTENDED;
          } else {
            io.setPIDEnabled(true);
            io.goToPos(targetPosition);
          }
        } else {
          io.setPIDEnabled(true);
          io.goToPos(targetPosition);
        }
      }
      case EXTENDED -> {
        io.setPIDEnabled(true);
        io.goToPos(EXTEND_POS);
      }
      case MANUAL -> {
        io.setPIDEnabled(false);
        io.setVoltageExtend(manualVoltage);
      }
    }
  }

  /** Creates an intake extension subsystem using the selected hardware abstraction. */
  public IntakeExtend(IntakeExtendIO io) {
    this.io = io;
  }

  /** Returns whether the extension is currently on the collapsed limit switch. */
  public boolean isHopperCollapsed() {
    return inputs.isCollapsed;
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
    return Commands.runOnce(
        () -> {
          io.resetExtendEncoder(pose);
          hasPose = true;
        },
        this);
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
    state = State.IDLE;
  }

  /** Creates a one-shot command that zeros the extension encoder. */
  public Command resetExtendPosition() {
    return Commands.runOnce(
        () -> {
          io.resetExtendEncoder(0.0);
          hasPose = true;
          state = State.COLLAPSED;
        },
        this);
  }

  /** Moves the extension to the configured deployed position and finishes at tolerance. */
  public Command moveToExtendedPosition() {
    return extendToAngle(EXTEND_POS).withName("moveToExtendedPosition");
  }

  /** Moves the extension to the configured collapsed position and finishes at tolerance. */
  public Command moveToCollapsedPosition() {
    return extendToAngle(COLLAPSE_POS).withName("moveToCollapsedPosition");
  }

  /** Creates a manual voltage command for operator-controlled extension movement. */
  public Command runIntakeExtendVolts(double volts) {
    return run(() -> {
          state = State.MANUAL;
          manualVoltage = volts;
        })
        .finallyDo(this::stopExtend);
  }

  /** Creates a command that keeps the extension stopped while scheduled. */
  public Command stopExtendingCommand() {
    return run(this::stopExtend);
  }

  /** Homes the extension by driving until the collapsed limit switch is reached. */
  public Command homeExtend() {
    return run(() -> state = State.HOMING)
        .beforeStarting(() -> Logger.recordOutput("Intake/IntakeExtend/HomeStarted", true))
        .until(() -> state == State.COLLAPSED)
        .finallyDo(this::stopExtend)
        .withName("homeExtend");
  }

  /** Homes first if needed, then moves the extension to the requested encoder angle. */
  public Command extendToAngle(double angle) {
    return run(() -> {
          targetPosition = angle;
          if (angle == COLLAPSE_POS) {
            if (state != State.COLLAPSED) {
              state = State.COLLAPSING;
            }
          } else if (angle == EXTEND_POS) {
            if (state != State.EXTENDED) {
              state = State.EXTENDING;
            }
          } else {
            if (inputs.extendPosition > angle) {
              state = State.COLLAPSING;
            } else {
              state = State.EXTENDING;
            }
          }
        })
        .until(
            () ->
                (state == State.COLLAPSED && angle == COLLAPSE_POS)
                    || (state == State.EXTENDED && angle == EXTEND_POS)
                    || (Math.abs(inputs.extendPosition - angle) <= POSITION_TOLERANCE))
        .finallyDo(this::stopExtend)
        .withName("extendToAngle");
  }

  // CAUTION: Do not use unless absolutely necessary, may cause unexpected behavior
  public void overrideState(State newState) {
    state = newState;
  }
}
