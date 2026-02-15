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
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.DoubleSupplier;
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
    this.intake = intake;
    this.driverController = driverController;
  }
}