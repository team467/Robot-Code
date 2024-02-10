package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Autos {
  private final Drive drive;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Arm arm;
  private final Intake intake;

  public Autos(Drive drive, Shooter shooter, Indexer indexer, Arm arm, Intake intake) {
    this.drive = drive;
    this.shooter = shooter;
    this.indexer = indexer;
    this.arm = arm;
    this.intake = intake;
  }

  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
  }

  public Command oneNoteAuto() {

    return shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC);
  }

  public Command twoNoteAuto(StartingPosition position) {

    return switch (position) {
      case LEFT -> shooter
          .shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)
          .andThen(
              Commands.parallel(
                  new StraightDriveToPose(
                      new Pose2d(
                          // TODO: Add the correct pose for the two note auto
                          ),
                      drive),
                  arm.toSetpoint(
                      new Rotation2d(-9000) // TODO: Add the correct setpoint for the two note auto
                      )))
          .andThen(Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
          .andThen(
              new StraightDriveToPose(
                  new Pose2d(
                      // TODO: Add the correct pose for the two note auto
                      ),
                  drive))
          .andThen(
              arm.toSetpoint(
                  new Rotation2d(-9000)
                  // TODO: Add the correct setpoint for the two note auto
                  ))
          .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC));
      case CENTER ->
      //              shooter
      //          .shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)
      (Commands.parallel(
              new StraightDriveToPose(new Pose2d(1.0, 1.0, new Rotation2d()), drive),
              arm.toSetpoint(
                  new Rotation2d(-9000) // TODO: Add the correct setpoint for the two note auto
                  )))
          .andThen(Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
          .andThen(
              new StraightDriveToPose(
                  new Pose2d(
                      // TODO: Add the correct pose for the two note auto
                      ),
                  drive))
          .andThen(
              arm.toSetpoint(
                  new Rotation2d(-9000)
                  // TODO: Add the correct setpoint for the two note auto
                  ))
          .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC));
      case RIGHT -> shooter
          .shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC)
          .andThen(
              Commands.parallel(
                  new StraightDriveToPose(
                      new Pose2d(
                          // TODO: Add the correct pose for the two note auto
                          ),
                      drive),
                  arm.toSetpoint(
                      new Rotation2d(-9000) // TODO: Add the correct setpoint for the two note auto
                      )))
          .andThen(Commands.parallel(intake.intake(), indexer.setIndexerPercentVelocity(0.25)))
          .andThen(
              new StraightDriveToPose(
                  new Pose2d(
                      // TODO: Add the correct pose for the two note auto
                      ),
                  drive))
          .andThen(
              arm.toSetpoint(
                  new Rotation2d(-9000)
                  // TODO: Add the correct setpoint for the two note auto
                  ))
          .andThen(shooter.shoot(ShooterConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC));
      default -> Commands.none();
    };
  }
}
