package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.subsystems.algae.AlgaeEffector;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Orchestrator {

  private final Drive drive;
  private final Elevator elevator;
  private final AlgaeEffector algaeEffector;
  private final CoralEffector coralEffector;
  private final RobotState robotState = RobotState.getInstance();
  private static final double L4_HEIGHT = ReefHeight.L4.height;
  private static final double L3_HEIGHT = ReefHeight.L3.height;
  private static final double L2_HEIGHT = ReefHeight.L2.height;
  private static final double L1_HEIGHT = ReefHeight.L1.height;
  private static final double ALGAE_HEIGHT_OFFSET = 0.0;

  public Orchestrator(
      Drive drive, Elevator elevator, AlgaeEffector algaeEffector, CoralEffector coralEffector) {
    this.drive = drive;
    this.elevator = elevator;
    this.algaeEffector = algaeEffector;
    this.coralEffector = coralEffector;
  }

  /**
   * Intakes coral after driving towards the nearest coral station and waiting until intake in
   * position.
   *
   * @return Command for intaking coral.
   */
  public Command intake() {
    return elevator
        .toSetpoint(ElevatorConstants.INTAKE_POSITION)
        .withTimeout(4)
        .until(() -> elevator.atSetpoint() && robotState.hopperSeesCoral)
        .andThen(coralEffector.intakeCoral());
  }

  /**
   * Places coral on branch after getting in position.
   *
   * @param level
   * @return Command for placing coral.
   */
  public Command placeCoral(int level) {
    return elevator
        .toSetpoint(getCoralHeight(level))
        .until(elevator::atSetpoint)
        .andThen(coralEffector.dumpCoral());
  }

  /**
   * Removes algae piece after getting in position for algae.
   *
   * @param level
   * @return Command to remove algae from reef.
   */
  public Command removeAlgae(int level) {
    return Commands.sequence(
        algaeEffector.stowArm().withTimeout(3),
        elevator
            .toSetpoint(getAlgaeHeight(level))
            .until(elevator::atSetpoint)
            .andThen(algaeEffector.removeAlgae())
            .finallyDo(algaeEffector::stowArm));
  }
  /**
   * Gets the level for the branch that we want.
   *
   * @param level The level that the coral is on.
   * @return Command for getting the coral height.
   */
  public double getCoralHeight(int level) {
    // The default branch we want
    return switch (level) {
      case 1 -> L1_HEIGHT;
      case 2 -> L2_HEIGHT;
      case 3 -> L3_HEIGHT;
      case 4 -> L4_HEIGHT;
      default -> 0.0;
    };
  }
  /**
   * Gets height of algae we want to extract.
   *
   * @param level The level that the algae is on.
   * @return Command for getting the algae height.
   */
  public double getAlgaeHeight(int level) {
    // The default branch we want
    double height =
        switch (level) {
          case 1, 2 -> L2_HEIGHT;
          case 3, 4 -> L3_HEIGHT;
          default -> 0.0;
        };
    return height + ALGAE_HEIGHT_OFFSET;
  }
}
