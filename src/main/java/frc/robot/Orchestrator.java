package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterLeadCompensator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Orchestrator {
  private final Drive drive;
  private final Shooter shooter;
  private final HopperBelt hopperBelt;
  private final Indexer indexer;
  private final Intake intake;
  private final RobotState robotState = RobotState.getInstance();
  private final ShooterLeadCompensator shooterLeadCompensator;

  public Orchestrator(
      Drive drive, HopperBelt hopperBelt, Shooter shooter, Indexer indexer, Intake intake) {
    this.drive = drive;
    this.hopperBelt = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.shooterLeadCompensator = new ShooterLeadCompensator(drive, shooter);
    this.intake = intake;
  }

  public void OrchestratorPeriodic() {
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    Logger.recordOutput(
        "Orchestrator/Target",
        new Pose2d(
            shootWhileDrivingResult.target().getX(),
            shootWhileDrivingResult.target().getY(),
            Rotation2d.fromDegrees(0)));
  }

  /** created command to shoot the balls so it runs the shooter, hopperBelt and indexer */
  public Command shootBalls() {
    return preloadBalls().andThen(shooter.setTargetVelocity(360)).onlyIf(() -> shooter.getSetpoint() > 1)
        .andThen(Commands.parallel(hopperBelt.start(), indexer.run()))
        .onlyWhile(
            () ->
                shooter.getSetpoint() > 0.1
                    && RobotState.getInstance().shooterAtSpeed
                    && RobotState.getInstance().isAlignedToHub)
        .finallyDo(() -> Commands.parallel(hopperBelt.stop(), preloadBalls()));
  }

  public Command preloadBalls() {
    return indexer.run().until(indexer::isSwitchPressed);
  }

  public Command alignAndShoot(DoubleSupplier xsupplier, DoubleSupplier ysupplier) {
    return Commands.parallel(shootBalls());
    // TODO: AIMING LOGIC
  }
}
