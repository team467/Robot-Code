package frc.robot.subsystems.intake.extend;

import static frc.robot.subsystems.intake.IntakeConstants.COLLAPSE_POS;
import static frc.robot.subsystems.intake.IntakeConstants.EXTEND_POS;
import static frc.robot.subsystems.intake.IntakeConstants.POSITION_TOLERANCE;
import static frc.robot.subsystems.intake.IntakeConstants.STALL_VELOCITY;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.IntakePosition;
import java.util.function.BooleanSupplier;
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
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/IntakeExtend/StalledExtend", stalledExtend);
    Logger.recordOutput("Intake/IntakeExtended/StalledCollapse", stalledCollapse);
    Logger.recordOutput("Intake/stallingExtend", isStallingExtend());
    Logger.recordOutput("Intake/stallingCollapse", isStallingCollapse());
    if (inputs.getExtendPos > EXTEND_POS / 2) {
      RobotState.getInstance().intakePosition = IntakePosition.STOWED;
    }
    if (inputs.getExtendPos <= EXTEND_POS / 2) {
      RobotState.getInstance().intakePosition = IntakePosition.DEPLOYED;
    }
    if (inputs.isCollapsed) {
      io.resetExtendEncoder(0.0);
    }
  }

  public IntakeExtend(IntakeExtendIO io, BooleanSupplier limitSwitchDisabled) {
    this.io = io;
    this.limitSwitchDisabled = limitSwitchDisabled;
  }

  public boolean isHopperCollapsed() {
    return io.isCollapsed();
  }

  public DoubleSupplier getAngle(){
    return () -> inputs.getExtendPos;
  }

  private boolean isStallingExtend() {
    // For our volts, we are not getting the right velocity
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts < -0.01;
  }

  private boolean isStallingCollapse() {
    return Math.abs(inputs.extendVelocity) < STALL_VELOCITY && inputs.extendVolts > 0.01;
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
