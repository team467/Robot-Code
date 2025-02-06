package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.algae.AlgaeEffector;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.vision.Vision;

public class Orchestrator {

  private final Drive drive;
  private final Elevator elevator;
  private final AlgaeEffector algaeEffector;
  private final CoralEffector coralEffector;
  private final Vision vision;
  private final RobotState robotState = RobotState.getInstance();

  public Orchestrator(
      Drive drive,
      Elevator elevator,
      AlgaeEffector algaeEffector,
      CoralEffector coralEffector,
      Vision vision) {
    this.drive = drive;
    this.elevator = elevator;
    this.algaeEffector = algaeEffector;
    this.coralEffector = coralEffector;
    this.vision = vision;
  }

  /**
   * Setting the elevator to duck position.
   *
   * @return Command for lowering the elevator.
   */
  public Command duck() {
    return elevator.toSetpoint(ElevatorConstants.DUCK_POSITION);
  }

  /**
   * Intakes coral after driving towards the nearest coral station and waiting until intake in
   * position.
   *
   * @return Command for intaking coral.
   */
  public Command intake() {
    return new StraightDriveToPose(getClosestCoralStationPosition(), drive)
        .andThen(
            elevator
                .toSetpoint(ElevatorConstants.INTAKE_POSITION)
                .withTimeout(4)
                .until(() -> elevator.atSetpoint() && robotState.hopperSeesCoral))
        .andThen(coralEffector.intakeCoral());
  }

  /**
   * Places coral on branch after getting in position.
   *
   * @param branchLeft
   * @param level
   * @return Command for placing coral.
   */
  public Command placeCoral(boolean branchLeft, int level) {

    return new StraightDriveToPose(getBranchPosition(branchLeft), drive)
        .andThen(elevator.toSetpoint(getCoralHeight(level)).until(elevator::atSetpoint))
        .andThen(coralEffector.dumpCoral());
  }

  /**
   * Removes algae piece after getting in position for algae.
   *
   * @param level
   * @return Command to remove algae from reef.
   */
  public Command removeAlgae(int level) {
    return new StraightDriveToPose(getBranchPosition(false), drive)
        .andThen(elevator.toSetpoint(getAlgaeHeight(level)))
        .until(elevator::atSetpoint)
        .andThen(algaeEffector.removeAlgae())
        .andThen(algaeEffector.stowArm());
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
      case 1 -> ReefHeight.L1.height;
      case 2 -> ReefHeight.L2.height;
      case 3 -> ReefHeight.L3.height;
      case 4 -> ReefHeight.L4.height;
      default -> 0.0;
    }; // subtract offset
  }

  /**
   * Gets height of algae we want to extract.
   *
   * @param level The level that the algae is on.
   * @return Command for getting the algae height.
   */
  public double getAlgaeHeight(int level) {
    // double Algae offset
    double offset = 0.0;
    // The default branch we want
    double height =
        switch (level) {
          case 1, 2 -> ReefHeight.L2.height;
          case 3, 4 -> ReefHeight.L3.height;
          default -> 0.0;
        };
    return height + offset;
  }

  /**
   * Gets position of the branch we want.
   *
   * @param branchLeft
   * @return Command for getting branch postion.
   */
  public Pose2d getBranchPosition(boolean branchLeft) {
    int branch = closestReefFace();
    if (!branchLeft) {
      branch++;
    }
    return Reef.branchPositions.get(branch).get(ReefHeight.L1).toPose2d();
  }

  /**
   * Gets the closest coral station position, either leftCenterFace or rightCenterFace.
   *
   * @return Command for getting the closest coral station.
   */
  public Pose2d getClosestCoralStationPosition() {

    return closerToLeftCoralStation() ? CoralStation.leftCenterFace : CoralStation.rightCenterFace;
  }

  public boolean closerToLeftCoralStation() {
    double distanceToLeftStation =
        Math.hypot(
            Math.abs(drive.getPose().minus(CoralStation.leftCenterFace).getX()),
            Math.abs(drive.getPose().minus(CoralStation.leftCenterFace).getY()));
    double distanceToRightStation =
        Math.hypot(
            Math.abs(drive.getPose().minus(CoralStation.rightCenterFace).getX()),
            Math.abs(drive.getPose().minus(CoralStation.rightCenterFace).getY()));
    return (distanceToLeftStation < distanceToRightStation);
  }

  public int closestReefFace() {
    double[] reefFaceDistances = new double[6];

    for (int i = 0; i < 6; i++) {
      reefFaceDistances[i] =
          Math.abs(drive.getPose().minus(FieldConstants.Reef.centerFaces[i]).getX());
    }
    int closestFace = 0;

    for (int i = 0; i < reefFaceDistances.length; i++) {
      if (reefFaceDistances[i] < closestFace) {
        closestFace = i;
      }
    }
    return closestFace + 1;
  }
}
