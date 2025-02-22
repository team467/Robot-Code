package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.lib.utils.ChoreoVariables;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AutosAlternate {
  private final Drive drive;

  public AutosAlternate(Drive drive) {
    this.drive = drive;
  }

  public Command zeroPiece() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(-1, 0, 0, drive));
  }

  public Command B1L() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    Supplier<Pose2d> L1 = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("L1"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(drive, L1));
  }

  public Command B1R() {
    Supplier<Pose2d> B = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("B"));
    Supplier<Pose2d> R1 = () -> AllianceFlipUtil.apply(ChoreoVariables.getPose("R1"));
    return Commands.runOnce(() -> drive.setPose(B.get()))
        .andThen(new StraightDriveToPose(drive, R1));
  }
}
