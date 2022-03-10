package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter2020;
import java.util.function.Supplier;

public class ShooterSetCMD extends CommandBase {
  private final Shooter2020 shooter2020;
  private final Supplier<Double> speedSupplier;

  public ShooterSetCMD(Shooter2020 shooter2020, Supplier<Double> speedSupplier) {
    this.shooter2020 = shooter2020;
    this.speedSupplier = speedSupplier;

    addRequirements(shooter2020);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter2020.setFlywheel(speedSupplier.get());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
