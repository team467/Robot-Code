package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.RobotState.ElevatorPosition;
import frc.robot.subsystems.algae.AlgaeEffector;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Orchestrator {
  private final Elevator elevator;
  private final AlgaeEffector algaeEffector;
  private final CoralEffector coralEffector;
  private final RobotState robotState = RobotState.getInstance();
  private static final double L4_HEIGHT = ReefHeight.L4.height;
  private static final double L3_HEIGHT = ReefHeight.L3.height;
  private static final double L2_HEIGHT = ReefHeight.L2.height;
  private static final double L1_HEIGHT = ReefHeight.L1.height;
  private static final double ALGAE_L2_HEIGHT = ReefHeight.ALGAE_LOW.height;
  private static final double ALGAE_L3_HEIGHT = ReefHeight.ALGAE_HIGH.height;

  public Orchestrator(Elevator elevator, AlgaeEffector algaeEffector, CoralEffector coralEffector) {
    this.elevator = elevator;
    this.algaeEffector = algaeEffector;
    this.coralEffector = coralEffector;
    robotState.elevatorPosition = ElevatorPosition.INTAKE;
  }

  /**
   * Intakes coral after driving towards the nearest coral station and waiting until intake in
   * position.
   *
   * @return Command for intaking coral.
   */
  public Command intake() {
    return moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)
        .until(elevator::limitSwitchPressed)
        .andThen(elevator.runPercent(-0.1))
        .withTimeout(1.0)
        .andThen(
            Commands.runOnce(
                () -> {
                  robotState.elevatorPosition = ElevatorPosition.INTAKE;
                }))
        .andThen(coralEffector.intakeCoral());
  }

  /**
   * Places coral on branch after getting in position.
   *
   * @param level
   * @return Command for placing coral.
   */
  public Command placeCoral(int level) {
    return moveElevatorToLevel(false, level).withTimeout(1).andThen(coralEffector.dumpCoral());
  }

  /**
   * Removes algae piece after getting in position for algae.
   *
   * @param level
   * @return Command to remove algae from reef.
   */
  public Command removeAlgae(int level) {
    return moveElevatorToLevel(true, level).andThen(algaeEffector.removeAlgae());
  }

  public Command moveElevatorToLevel(boolean algae, int level) {
    Command moveElevator =
        algae
            ? moveElevatorToSetpoint(getAlgaeHeight(level))
            : moveElevatorToSetpoint(getCoralHeight(level));
    return moveElevator.andThen(
        Commands.runOnce(
            () -> {
              switch (level) {
                case 1:
                  robotState.elevatorPosition = ElevatorPosition.L1;
                  break;
                case 2:
                  robotState.elevatorPosition =
                      algae ? ElevatorPosition.ALGAE_L2 : ElevatorPosition.L2;
                  break;
                case 3:
                  robotState.elevatorPosition =
                      algae ? ElevatorPosition.ALGAE_L3 : ElevatorPosition.L3;
                  break;
                case 4:
                  robotState.elevatorPosition = ElevatorPosition.L4;
                  break;
              }
            }));
  }

  public Command dumpCoralAndHome() {
    return coralEffector
        .dumpCoral()
        .andThen(Commands.waitSeconds(0.4))
        .andThen(
            moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)
                .until(elevator::limitSwitchPressed));
  }

  public Command moveElevatorToSetpoint(double setpoint) {

    return (Commands.either(
        algaeEffector
            .stowArm()
            .until(algaeEffector::isStowed)
            .andThen( // Stow algae if moving the elevator
                elevator
                    .toSetpoint(setpoint)
                    .withTimeout(0.01)
                    .andThen(elevator.toSetpoint(setpoint).until(elevator::atSetpoint)))
            .andThen(elevator.setHoldPosition(elevator.getPosition())),
        elevator
            .toSetpoint(setpoint)
            .withTimeout(0.01)
            .andThen(elevator.toSetpoint(setpoint).until(elevator::atSetpoint))
            .andThen(elevator.setHoldPosition(elevator.getPosition())),
        // If we are moving from one algae position to another, we don't need to make sure that the
        // algae effector is stowed
        () ->
            !((setpoint == ALGAE_L2_HEIGHT || setpoint == ALGAE_L3_HEIGHT)
                && (robotState.elevatorPosition == ElevatorPosition.ALGAE_L2
                    || robotState.elevatorPosition == ElevatorPosition.ALGAE_L3))));
  }

  public Command scoreL1() {
    return Commands.parallel(coralEffector.dumpCoral(), algaeEffector.removeAlgae());
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
    return switch (level) {
      case 2 -> ALGAE_L2_HEIGHT;
      case 3 -> ALGAE_L3_HEIGHT;
      default -> 0.0;
    };
  }
}
