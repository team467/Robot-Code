package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algae.AlgaeEffector;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.HashMap;
import java.util.Map;

public class orchestrator {

  private final Drive drive;
  private final Elevator elevator;
  private final AlgaeEffector algaeEffector;
  private final CoralEffector coralEffector;
  private final Vision vision;
  private final RobotState robotState = RobotState.getInstance();

  public orchestrator(Drive drive, Elevator elevator, AlgaeEffector algaeEffector,
      CoralEffector coralEffector, Vision vision) {
    this.drive = drive;
    this.elevator = elevator;
    this.algaeEffector = algaeEffector;
    this.coralEffector = coralEffector;
    this.vision = vision;
  }

  public Command duck() {
    return Commands.run(() -> {
      robotState.duck = true;
    }).andThen(elevator.toSetpoint(ElevatorConstants.DUCK_POSITION));
  }
  public Command intake() {
  return elevator.toSetpoint(ElevatorConstants.INTAKE_POSITION).withTimeout(5).andThen(coralEffector.intakeCoral());
  }
}
