package frc.robot.subsystems.drive.controllers.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.AutoAimController;
import frc.robot.subsystems.drive.controllers.TeleopDriveController;
import java.util.function.Supplier;

public class AutoAim extends Command {
  private final TeleopDriveController teleopController;
  private final AutoAimController aimController;

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;

  public AutoAim(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Boolean> robotRelativeOverride) {
    teleopController = new TeleopDriveController(drive);
    aimController = new AutoAimController();

    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    teleopController.acceptDriveInput(
        leftXSupplier.get(), leftYSupplier.get(), rightXSupplier.get());
    ChassisSpeeds teleopControllerOutput = teleopController.update();
    double aimedControllerOutput = aimController.update();
    ChassisSpeeds adjustedSpeeds =
        new ChassisSpeeds(
            teleopControllerOutput.vxMetersPerSecond,
            teleopControllerOutput.vyMetersPerSecond,
            aimedControllerOutput);
    drive.runVelocity(adjustedSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
