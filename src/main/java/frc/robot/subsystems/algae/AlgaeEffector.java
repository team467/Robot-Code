package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEffector extends SubsystemBase {
  private final AlgaeEffectorIO io;
  private PIDController pivotFeedback =
      new PIDController(AlgaeEffectorConstants.PIVOT_KP, 0, AlgaeEffectorConstants.PIVOT_KD);

  public AlgaeEffector(AlgaeEffectorIO io) {
    this.io = io;
  }

  public Command extendArm() {
    return Commands.startEnd(
            () -> io.setPivotVolts(AlgaeEffectorConstants.EXTEND_VOLTAGE),
            () -> io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOlTAGE),
            this)
        .withTimeout(2);
  }

  public Command retractArm() {
    return Commands.startEnd(
            () -> io.setPivotVolts(AlgaeEffectorConstants.RETRACT_VOLTAGE),
            () -> io.setPivotVolts(AlgaeEffectorConstants.ZERO_VOlTAGE),
            this)
        .withTimeout(2);
  }

  public Command startRemoval() {
    return Commands.startEnd(
        () -> io.setRemovalVolts(AlgaeEffectorConstants.REMOVAL_VOLTAGE),
        () -> io.setRemovalVolts(AlgaeEffectorConstants.ZERO_VOlTAGE),
        this);
  }

  public Command stopRemoval() {
    return this.runOnce(() -> io.setRemovalVolts(AlgaeEffectorConstants.ZERO_VOlTAGE));
  }
}
