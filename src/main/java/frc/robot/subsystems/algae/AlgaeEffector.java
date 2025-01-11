package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeEffector extends SubsystemBase {
  private final AlgaeEffectorIO io;
  private PIDController pivotFeedback =
      new PIDController(AlgaeEffectorConstants.PIVOT_KP, 0, AlgaeEffectorConstants.PIVOT_KD);

  public AlgaeEffector(AlgaeEffectorIO io) {
    this.io = io;
  }

  public Command extendArm(double length) {
    return this.runOnce(() -> io.setPivotVolts(AlgaeEffectorConstants.EXTEND_VOLTAGE));
  }

  public Command retractArm(double length) {
    return this.runOnce(() -> io.setPivotVolts(AlgaeEffectorConstants.RETRACT_VOLTAGE));
  }
}
