package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import java.util.logging.Level;

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

  public Command duck() {
    return Commands.run(
            () -> {
              robotState.duck = true;
            })
        .andThen(elevator.toSetpoint(ElevatorConstants.DUCK_POSITION));
  }

  public Command intake() {
    //get the closest Coral Station
    boolean leftStation = false;
    return new StraightDriveToPose(getCoralStationPosition(leftStation), drive)
        .andThen(
            elevator
                .toSetpoint(ElevatorConstants.INTAKE_POSITION)
                .withTimeout(5)
                .until(() -> elevator.atSetpoint() && robotState.hopperSeesCoral))
        .andThen(coralEffector.intakeCoral());
  }

  public Command placeCoral(boolean branchLeft, int level) {

    return new StraightDriveToPose(getBranchPosition(branchLeft), drive)
        .andThen(elevator.toSetpoint(getCoralHeight(level)).until(elevator::atSetpoint))
        .andThen(coralEffector.dumpCoral());
  }
  public Command removeAlgae(int level){
    return new StraightDriveToPose(getBranchPosition(false), drive).andThen(elevator.toSetpoint(getAlgaeHeight(
        level))).until(elevator::atSetpoint).andThen(algaeEffector.removeAlgae()).andThen(algaeEffector.stowArm());
  }

  public double getCoralHeight(int level) {
    //The default branch we want
    return switch (level) {
        case 1 -> ReefHeight.L1.height;
        case 2 -> ReefHeight.L2.height;
        case 3 -> ReefHeight.L3.height;
        case 4 -> ReefHeight.L4.height;
        default -> 0.0;
    }; // subtract offset
  }
  public double getAlgaeHeight(int level){
      //double Algae offset
      double offset = 0.0;
      //The default branch we want
      double height = switch (level) {
          case 1, 2 -> ReefHeight.L2.height;
          case 3, 4 -> ReefHeight.L3.height;
          default -> 0.0;
      };
      return height + offset;
  }
  public Pose2d getBranchPosition(boolean branchLeft){
    //get face
    int branch = 6; //face
    if(!branchLeft){
      branch++;
    }
    return Reef.branchPositions.get(branch).get(ReefHeight.L1).toPose2d();
  }
  public Pose2d getCoralStationPosition(boolean leftStation){
    return leftStation ? CoralStation.leftCenterFace : CoralStation.rightCenterFace;
  }}
