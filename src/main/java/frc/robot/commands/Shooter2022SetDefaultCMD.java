package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter2022;

public class Shooter2022SetDefaultCMD extends InstantCommand {
  private final Shooter2022 shooter;
  private final Command defaultCommand;

  public Shooter2022SetDefaultCMD(Shooter2022 shooter, Command defaultCommand) {
    this.shooter = shooter;
    this.defaultCommand = defaultCommand;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setDefaultCommand(defaultCommand);
  }
}
