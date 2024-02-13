package frc.robot.subsystems.drive.controllers.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.controllers.TeleopDriveController;
import java.util.function.Supplier;

public class TeleopDrive extends Command {
  private final TeleopDriveController controller;

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;

  public TeleopDrive(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Boolean> robotRelativeOverride) {
    controller = new TeleopDriveController(drive);

    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    controller.acceptDriveInput(leftXSupplier.get(), leftYSupplier.get(), rightXSupplier.get());
    drive.runVelocity(controller.update());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
