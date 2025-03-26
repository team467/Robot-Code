package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.RobotState.ElevatorPosition;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.subsystems.fastalgae.FastAlgaeEffector;

public class Orchestrator {
  private final Drive drive;
  private final Elevator elevator;
  private final FastAlgaeEffector fastAlgaeEffector;
  private final CoralEffector coralEffector;
  private final RobotState robotState = RobotState.getInstance();
  private static final double L4_HEIGHT = ReefHeight.L4.height;
  private static final double L3_HEIGHT = ReefHeight.L3.height;
  private static final double L2_HEIGHT = ReefHeight.L2.height;
  private static final double L1_HEIGHT = ReefHeight.L1.height;
  private static final double ALGAE_L2_ANGLE = 0;
  private static final double ALGAE_L3_ANGLE = 0;

  public Orchestrator(
      Elevator elevator,
      FastAlgaeEffector fastAlgaeEffector,
      CoralEffector coralEffector,
      Drive drive) {
    this.elevator = elevator;
    this.fastAlgaeEffector = fastAlgaeEffector;
    this.coralEffector = coralEffector;
    this.drive = drive;
    robotState.elevatorPosition = ElevatorPosition.INTAKE;
  }

  /**
   * Intakes coral after driving towards the nearest coral station and waiting until intake in
   * position.
   *
   * @return Command for intaking coral.
   */
  public Command intake() {
    return Commands.parallel(
            moveElevatorToSetpoint(ElevatorConstants.INTAKE_POSITION)
                .until(elevator::limitSwitchPressed)
            //        .andThen(elevator.runPercent(-0.1))
            //        .withTimeout(1.0)
            //        .andThen(
            //            Commands.runOnce(
            //                () -> {
            //                  robotState.elevatorPosition = ElevatorPosition.INTAKE;
            //                }))
            ,
            coralEffector.intakeCoral())
        .until(coralEffector::hasCoral);
  }

  /**
   * Places coral on branch after getting in position.
   *
   * @param level
   * @return Command for placing coral.
   */
  public Command placeCoral(int level) {
    return moveElevatorToLevel(level).withTimeout(1).andThen(coralEffector.dumpCoral());
  }

  public Command removeAlgaeAndPlaceCoral(int level) {
    return Commands.deadline(placeCoral(level), removeAlgaeAuto());
  }
  /**
   * Removes algae piece after getting in position for algae.
   *
   * @return Command to remove algae from reef.
   */
  public Command removeAlgaeAuto() {
    if (RobotState.getInstance().ClosestReefFace % 2 == 0) {
      return fastAlgaeEffector.removeAlgaeHigh(); // remove algae high/low
    } else {
      return fastAlgaeEffector.removeAlgaeLow(); // remove algae high/low
    }
  }

  public Command removeAlgae(int level) {
    // return moveElevatorToLevel(true, level).andThen(algaeEffector.removeAlgae());
    if (level == 2) {
      return fastAlgaeEffector.removeAlgaeLow();
    } else if (level == 3) {
      return fastAlgaeEffector.removeAlgaeHigh();
    } else {
      return null;
    }
  }

  public Command moveElevatorToLevel(int level) {
    return moveElevatorToSetpoint(getCoralHeight(level))
        .andThen(
            Commands.runOnce(
                () -> {
                  switch (level) {
                    case 1:
                      robotState.elevatorPosition = ElevatorPosition.L1;
                      break;
                    case 2:
                      robotState.elevatorPosition = ElevatorPosition.L2;
                      break;
                    case 3:
                      robotState.elevatorPosition = ElevatorPosition.L3;
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
        .andThen(Commands.runOnce(() -> drive.run(Commands::none)))
        .andThen(Commands.waitSeconds(0.4))
        .andThen(moveElevatorToLevel(1).until(elevator::limitSwitchPressed));
  }

  public Command moveElevatorToSetpoint(double setpoint) {
    return elevator
        .toSetpoint(setpoint)
        .withTimeout(0.001)
        .andThen(elevator.toSetpoint(setpoint).until(elevator::atSetpoint));
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
      case 2 -> ALGAE_L2_ANGLE;
      case 3 -> ALGAE_L3_ANGLE;
      default -> 0.0;
    };
  }

  public Command moveElevatorBasedOnDistance(Supplier<Pose2d> targetPose) {
    return elevator.toSetpoint(
        setpointFromDistance(distanceToSetpoint(targetPose, drive::getPose)));
  }

  public DoubleSupplier distanceToSetpoint(Supplier<Pose2d> targetPose, Supplier<Pose2d> pose) {
    return () -> targetPose.get().getTranslation().getDistance(pose.get().getTranslation());
  }

  public DoubleSupplier setpointFromDistance(DoubleSupplier distance) {
    return () -> {
      double setpoint = (0.368) * Math.pow(0.1, distance.getAsDouble()) + 0.432;
      if (setpoint >= 0.7605) {
        setpoint = 0.7605;
      }
      return setpoint;
    };
  }
}
