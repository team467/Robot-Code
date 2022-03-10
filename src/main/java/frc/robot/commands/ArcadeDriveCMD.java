package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class ArcadeDriveCMD extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Double> speedSupplier;
  private final Supplier<Double> rotateSupplier;

  public ArcadeDriveCMD(
      Drivetrain drivetrain, Supplier<Double> speedSupplier, Supplier<Double> rotateSupplier) {
    this.drivetrain = drivetrain;
    this.speedSupplier = speedSupplier;
    this.rotateSupplier = rotateSupplier;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    if (RobotConstants.get().driveUsePID()) {
      drivetrain.resetLeftPID();
      drivetrain.resetRightPID();
    }
  }

  @Override
  public void execute() {
    drivetrain.arcadeDrive(speedSupplier.get(), rotateSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
