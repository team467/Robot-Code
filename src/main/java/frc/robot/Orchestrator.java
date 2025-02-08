package frc.robot;

import static frc.robot.FieldConstants.Reef.branchPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.CoralStation;
import frc.robot.FieldConstants.Reef;
import frc.robot.FieldConstants.ReefHeight;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.auto.StraightDriveToPose;
import frc.robot.subsystems.algae.AlgaeEffector;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;

public class Orchestrator {

  private final Drive drive;
  private final Elevator elevator;
  private final AlgaeEffector algaeEffector;
  private final CoralEffector coralEffector;
  private final RobotState robotState = RobotState.getInstance();
  @AutoLogOutput private Pose2d desiredCoralPose;
  @AutoLogOutput private int branchIndex;
  @AutoLogOutput private int closestReefFace;
  @AutoLogOutput private Pose2d closestReefFacePose;
  @AutoLogOutput private double[] reefFaceDistances = new double[6];

  private static final double CORAL_EFFECTOR_OFFSET = 12;
  private static final double STATION_TO_REEF_DISTANCE = 18.375;

  public Orchestrator(
      Drive drive, Elevator elevator, AlgaeEffector algaeEffector, CoralEffector coralEffector) {
    this.drive = drive;
    this.elevator = elevator;
    this.algaeEffector = algaeEffector;
    this.coralEffector = coralEffector;
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
    return Commands.defer(
            () ->
                new DriveToPose(drive, getBranchPosition(branchLeft, closestReefFace()))
                    .withTimeout(5),
            Set.of(drive))
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
    return new DriveToPose(drive, FieldConstants.Reef.centerFaces[closestReefFace()])
        .andThen(elevator.toSetpoint(getAlgaeHeight(level)))
        .until(elevator::atSetpoint)
        .andThen(algaeEffector.removeAlgae())
        .finallyDo(algaeEffector::stowArm);
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
    };
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
  public Pose2d getBranchPosition(boolean branchLeft, int closestReefFace) {
    int branch = closestReefFace * 2;
    if (branchLeft) {
      branch++;
    }
    branchIndex = branch;
    Pose2d branchPose = branchPositions.get(branch).get(ReefHeight.L1).toPose2d();
    double desiredRotation = branchPose.getRotation().getDegrees() + 180;
    desiredCoralPose =
        new Pose2d(
            branchPose.getX()
                - Units.inchesToMeters(STATION_TO_REEF_DISTANCE)
                    * Math.cos(Units.degreesToRadians(desiredRotation))
                - Units.inchesToMeters(CORAL_EFFECTOR_OFFSET)
                    * Math.sin(Units.degreesToRadians(desiredRotation)),
            branchPose.getY()
                - Units.inchesToMeters(STATION_TO_REEF_DISTANCE)
                    * Math.sin(Units.degreesToRadians(desiredRotation))
                + Units.inchesToMeters(STATION_TO_REEF_DISTANCE)
                    * Math.cos(Units.degreesToRadians(desiredRotation)),
            Rotation2d.fromDegrees(desiredRotation));
    return desiredCoralPose;
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
            Math.abs(drive.getPose().getX() - (CoralStation.leftCenterFace).getX()),
            Math.abs(drive.getPose().getY() - (CoralStation.leftCenterFace).getY()));
    double distanceToRightStation =
        Math.hypot(
            Math.abs(drive.getPose().getX() - (CoralStation.rightCenterFace).getX()),
            Math.abs(drive.getPose().getY() - (CoralStation.rightCenterFace).getY()));
    return (distanceToLeftStation < distanceToRightStation);
  }

  public int closestReefFace() {
    double[] reefFaceDistances = new double[6];

    for (int i = 0; i < 6; i++) {
      reefFaceDistances[i] =
          Math.hypot(
              Math.abs(drive.getPose().getX() - (Reef.centerFaces[i]).getX()),
              Math.abs(drive.getPose().getY() - (Reef.centerFaces[i]).getY()));
      this.reefFaceDistances[i] = reefFaceDistances[i];
    }
    int closestFace = 0;

    for (int i = 0; i < reefFaceDistances.length; i++) {
      if (reefFaceDistances[i] < reefFaceDistances[closestFace]) {
        closestFace = i;
      }
    }
    closestReefFace = closestFace;
    closestReefFacePose = Reef.centerFaces[closestFace];
    return closestFace;
  }
}
