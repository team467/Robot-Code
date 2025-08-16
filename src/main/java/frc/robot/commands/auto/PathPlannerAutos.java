package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.Orchestrator;
import frc.robot.commands.drive.FieldAlignment;
import frc.robot.subsystems.coral.CoralEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.fastalgae.FastAlgaeEffector;
import java.io.IOException;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

public class PathPlannerAutos {
  private final Drive drive;
  private final Orchestrator orchestrator;
  private final FieldAlignment fieldAlignment;
  private final CoralEffector coral;
  private final FastAlgaeEffector algae;
  private final Trigger hopperSeesCoral;
  private static final Supplier<Pose2d> A =
      () -> AllianceFlipUtil.apply(new Pose2d(new Translation2d(7.30, 6.14), new Rotation2d(3.14)));
  private static final Supplier<Pose2d> B =
      () -> AllianceFlipUtil.apply(new Pose2d(new Translation2d(7.30, 3.99), new Rotation2d(3.14)));
  private static final Supplier<Pose2d> C =
      () -> AllianceFlipUtil.apply(new Pose2d(new Translation2d(7.30, 1.89), new Rotation2d(3.14)));

  public PathPlannerAutos(
      Drive drive,
      Orchestrator orchestrator,
      FieldAlignment fieldAlignment,
      CoralEffector coral,
      FastAlgaeEffector algae) {
    this.drive = drive;
    this.orchestrator = orchestrator;
    this.fieldAlignment = fieldAlignment;
    this.coral = coral;
    this.algae = algae;
    hopperSeesCoral = new Trigger(coral::hopperSeesCoral).debounce(0.2);
    ;
  }

  public Command runPath(String pathName) {
    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (IOException e) {
      throw new RuntimeException(e);
    } catch (ParseException e) {
      throw new RuntimeException(e);
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }

  public Command C4Coral() {
    return Commands.sequence(
        fieldAlignment.alignToReef(false),
        runPath("1LB-INT"),
        fieldAlignment.alignToReef(true),
        fieldAlignment.alignToCoralStation(),
        fieldAlignment.alignToReef(false),
        fieldAlignment.alignToCoralStation(),
        runPath("INT-3"),
        fieldAlignment.alignToReef(false));
  }
}
