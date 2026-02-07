package frc.robot;

import static frc.robot.FieldConstants.Hub;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperbelt.HopperBelt;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterLeadCompensator;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Orchestrator {
  private final double FRONT_HUB_OFFSET = -1.0;
  private final double FRONT_HUB_SHOOTER_VELOCITY = 0.0;
  private final Drive drive;
  private final Shooter shooter;
  private final HopperBelt hopperBelt;
  private final Indexer indexer;
  private final Intake intake;
  private final RobotState robotState = RobotState.getInstance();
  private final ShooterLeadCompensator shooterLeadCompensator;
  private final CommandXboxController driverController;

  public Orchestrator(
      Drive drive,
      HopperBelt hopperBelt,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      CommandXboxController driverController) {
    this.drive = drive;
    this.hopperBelt = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.shooterLeadCompensator = new ShooterLeadCompensator(drive, shooter);
    this.intake = intake;
    this.driverController = driverController;
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
  //TODO move to drive commands/shooter?
  public Command driveToHub(){
    return new DriveToPose(drive, AllianceFlipUtil.apply(Hub.nearFace.transformBy(new Transform2d(0.0, -FRONT_HUB_OFFSET, Rotation2d.fromDegrees(0)))));
  }

  public Command shootAtHub() {
    return shooter.setTargetVelocity(FRONT_HUB_SHOOTER_VELOCITY);
  }
  
  public Command shootBalls() {
    return preloadBalls()
        .andThen(shooter.setTargetVelocity(360)) // change
        .onlyIf(() -> shooter.getSetpoint() == 0)
        .andThen(Commands.parallel(hopperBelt.start(), indexer.run()))
        .onlyWhile(
            () ->
                shooter.getSetpoint() > 0.1
                    && RobotState.getInstance().shooterAtSpeed
                    && RobotState.getInstance().isAlignedToHub)
        .finallyDo(() -> Commands.parallel(hopperBelt.stop(), preloadBalls()));
  }

  //Toggle
  public Command shootBallsAtDistance(){
    return Commands.none();
    //TODO: Stop drive, face reef, shoot sequence
  }
  /** created command to shoot the balls so it runs the shooter, hopperBelt and indexer */
  public Command shootBallsWithDrive() {
    return preloadBalls()
        .andThen(
            Commands.parallel(
                hopperBelt.start(),
                indexer.run(),
                shooter.setTargetDistance(() -> shooterLeadCompensator.shootWhileDriving(Hub.innerCenterPoint.toTranslation2d()).distance())))
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
 public Command alignAndShoot(
      DoubleSupplier xsupplier,
      DoubleSupplier ysupplier,
      Supplier<Rotation2d> rotationSupplier,
      Drive drive) {
    return Commands.sequence(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> driverController.getLeftX(),
            () -> driverController.getLeftY(),
            AllianceFlipUtil.apply(Hub.blueCenter).minus(drive.getPose().getTranslation()).getAngle()),
        shootBalls());

    // TODO: AIMING LOGIC
  }
  public Command alignAndShootWhileDriving(DoubleSupplier xsupplier, DoubleSupplier ysupplier) {
    return Commands.parallel(shootBallsWithDrive());
    //    shooterLeadCompensator.shootWhileDriving(Hub.innerCenterPoint.toTranslation2d()).target();
    // TODO: AIMING LOGIC
  }
}

// shooterLeadCompensator.shootWhileDriving(Hub.innerCenterPoint.toTranslation2d()).target())
// shootWhileDriving rn is just a placeholder method so dont use this at all
// you aren't trying to go to the center...
// what this code would do was it would drive to the literal center of the hub we are// what is .target()

// here's what the command would actually do
// first, get the radius of the shot area that we can make form the center of the hub
// then move to the closest position on the radius of the half circle
// then rotate while moving
// then shoot
